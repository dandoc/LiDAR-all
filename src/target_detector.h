#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace Watch {
inline constexpr uint32_t MIN_DEPTH_MM=1500, INVALID_DEPTH_MM=964000;
struct Pixel { float x=0,y=0,z=0; uint32_t depth=0; };
// Geometry labels are per-frame; BoxTracker assigns high-bit persistent box IDs.
struct Target {
    uint32_t id=0; float x=0,y=0,z=0;
    uint32_t count=0,age=1; bool is_new=false;
    float width=0,height=0,extent_z=0,nearest=0;
    uint32_t flags=1; // 1 geometry, 2 box, 4 confirmed, 8 predicted (no observed points)
};
struct AutoStatus {
    uint32_t components=0,accepted=0,rejected=0,edge_checks=0;
    uint32_t labelled=0,valid=0; float processing_ms=0;
    uint32_t plane_fits=0,split_checks=0,split_objects=0,local_walls=0;
};
struct Result {
    unsigned background_frames=0; bool ready=true;
    std::vector<uint32_t> labels; std::vector<Target> targets;
    AutoStatus automatic;
};
struct Parameters {
    // Provisional tuning values, not a sensor accuracy specification.
    float link_base_m=.04f, link_per_m=.02f, link_max_m=.20f;
    unsigned min_points=8;
    // Experimental observed-point envelope calibrated to frames 1276/1622.
    // Physical box is 12 x 8 x 6 cm; these are NOT its physical dimensions.
    // No fixed spatial ROI or reference-frame subtraction is used at runtime.
    float box_min_width=.16f,box_max_width=.38f;
    float box_min_height=.08f,box_max_height=.24f;
    float box_seed=.06f,box_grow=.05f,box_max_front=.14f;
    // Require two observed dimensions; retain arbitrarily large walls.
    float min_long_span_m=.08f, min_second_span_m=.04f;
};
class Detector {
    Parameters p_;
    static bool valid(const Pixel& p) {
        return p.depth>=MIN_DEPTH_MM && p.depth<INVALID_DEPTH_MM &&
            std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&p.z>0;
    }
    struct Plane {
        double a=0,b=0,c=0,inv=1;
        double front(const Pixel& q)const{return (a*q.x+b*q.y+c-q.z)*inv;}
    };
    // Fit the surrounding ring only. No per-window RANSAC.
    bool ringPlane(const std::vector<Pixel>& pts,const std::vector<size_t>& ids,Plane& plane)const {
        if(ids.size()<16)return false;
        for(int pass=0;pass<2;++pass){
            double x=0,y=0,z=0,n=0;
            for(auto i:ids){const auto&q=pts[i];if(pass&&std::abs(plane.front(q))>.025)continue;x+=q.x;y+=q.y;z+=q.z;++n;}
            if(n<16||n<ids.size()*.7)return false;
            x/=n;y/=n;z/=n;
            double xx=0,xy=0,yy=0,xz=0,yz=0;
            for(auto i:ids){const auto&q=pts[i];if(pass&&std::abs(plane.front(q))>.025)continue;
                double dx=q.x-x,dy=q.y-y,dz=q.z-z;xx+=dx*dx;xy+=dx*dy;yy+=dy*dy;xz+=dx*dz;yz+=dy*dz;
            }
            double det=xx*yy-xy*xy;if(det<1e-10)return false;
            plane.a=(xz*yy-yz*xy)/det;plane.b=(yz*xx-xz*xy)/det;plane.c=z-plane.a*x-plane.b*y;
            plane.inv=1/std::sqrt(1+plane.a*plane.a+plane.b*plane.b);
        }
        if(plane.c<=0||plane.inv<.65)return false;
        double sum=0;unsigned n=0;
        for(auto i:ids){double f=plane.front(pts[i]);if(std::abs(f)<=.025){sum+=f*f;++n;}}
        return n>=ids.size()*.75&&std::sqrt(sum/std::max(1u,n))<=.016;
    }
    Target describe(const std::vector<Pixel>& pts,const std::vector<size_t>& ids)const {
        Target t;if(ids.empty())return t;
        float lo[3]={1e30f,1e30f,1e30f},hi[3]={-1e30f,-1e30f,-1e30f};double sums[3]={};double nearest=1e100;
        for(auto i:ids){const auto&q=pts[i];float a[3]={q.x,q.y,q.z};
            for(int k=0;k<3;++k){lo[k]=std::min(lo[k],a[k]);hi[k]=std::max(hi[k],a[k]);sums[k]+=a[k];}
            nearest=std::min(nearest,std::sqrt(double(q.x)*q.x+double(q.y)*q.y+double(q.z)*q.z));
        }
        t.count=ids.size();t.x=sums[0]/ids.size();t.y=sums[1]/ids.size();t.z=sums[2]/ids.size();
        t.width=hi[0]-lo[0];t.height=hi[1]-lo[1];t.extent_z=hi[2]-lo[2];t.nearest=nearest;return t;
    }
    bool boxSize(const Target&t)const {
        return t.count>=6&&t.width>=p_.box_min_width&&t.width<=p_.box_max_width&&
            t.height>=p_.box_min_height&&t.height<=p_.box_max_height&&t.extent_z<=.12f;
    }
    void splitBoxes(unsigned w,unsigned h,const std::vector<Pixel>& pts,Result& out)const {
        if(w<8||h<8)return;
        // Angular spacing is estimated from valid adjacent rays, not the cropped XYZ span.
        std::vector<float> dx,dy;
        for(unsigned y=0;y<h;++y)for(unsigned x=0;x<w;++x){size_t i=size_t(y)*w+x;if(!valid(pts[i]))continue;
            if(x+1<w&&valid(pts[i+1]))dx.push_back(std::abs(pts[i].x/pts[i].z-pts[i+1].x/pts[i+1].z));
            if(y+1<h&&valid(pts[i+w]))dy.push_back(std::abs(pts[i].y/pts[i].z-pts[i+w].y/pts[i+w].z));
        }
        if(dx.empty()||dy.empty())return;
        auto med=[](std::vector<float>&v){auto m=v.begin()+v.size()/2;std::nth_element(v.begin(),m,v.end());return *m;};
        float ax=med(dx),ay=med(dy);if(ax<1e-5||ay<1e-5)return;
        const auto labels=out.labels;const size_t original=out.targets.size();
        struct Candidate{std::vector<size_t> ids;uint32_t parent;float score;};std::vector<Candidate> proposals;
        for(int cy=2;cy<int(h)-2;cy+=3)for(int cx=2;cx<int(w)-2;cx+=3){
            size_t center=size_t(cy)*w+cx;uint32_t parent=labels[center];if(!parent)continue;
            // Keep the measured footprint and a margin inside the core; fit only outside it.
            int rx=std::max(2,int(std::ceil(.30f/(ax*pts[center].z))));
            int ry=std::max(2,int(std::ceil(.24f/(ay*pts[center].z))));
            if(rx>12||ry>12)continue;
            int x0=cx-rx,x1=cx+rx,y0=cy-ry,y1=cy+ry;
            if(x0<3||y0<3||x1+3>=int(w)||y1+3>=int(h))continue;
            std::vector<size_t> ring;
            for(int y=y0-3;y<=y1+3;++y)for(int x=x0-3;x<=x1+3;++x){
                if(x>=x0-1&&x<=x1+1&&y>=y0-1&&y<=y1+1)continue;
                size_t i=size_t(y)*w+x;if(valid(pts[i]))ring.push_back(i);
            }
            ++out.automatic.plane_fits;Plane plane;if(!ringPlane(pts,ring,plane))continue;
            unsigned total[4]={},support[4]={};
            for(auto i:ring){int x=i%w,y=i/w,k=-1;if(y>=y0&&y<=y1)k=x<x0?0:1;else if(x>=x0&&x<=x1)k=y<y0?2:3;
                if(k<0)continue;
                ++total[k];if(std::abs(plane.front(pts[i]))<=.025)++support[k];}
            unsigned sides=0;for(int k=0;k<4;++k)if(support[k]>=4&&support[k]>=.75*total[k])++sides;
            if(sides<3)continue;
            ++out.automatic.local_walls;
            // Cheap evidence gate: at least two strong points before region growth.
            unsigned strong=0;size_t seed=0;float best=0;
            for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x){size_t i=size_t(y)*w+x;if(labels[i]!=parent)continue;
                float f=plane.front(pts[i]);if(f>=p_.box_seed&&f<=p_.box_max_front){++strong;if(f>best){best=f;seed=i;}}}
            if(strong<2)continue;
            ++out.automatic.split_checks;
            std::vector<size_t> region{seed};std::vector<uint8_t> seen((2*rx+1)*(2*ry+1),0);
            auto index=[&](int x,int y){return (y-y0)*(2*rx+1)+x-x0;};seen[index(seed%w,seed/w)]=1;
            bool edge=false;strong=0;std::vector<float> depths;
            for(size_t n=0;n<region.size();++n){size_t i=region[n];int x=i%w,y=i/w;
                if(x==x0||x==x1||y==y0||y==y1)edge=true;
                float f=plane.front(pts[i]);depths.push_back(f);if(f>=p_.box_seed)++strong;
                for(int j=-1;j<=1;++j)for(int k=-1;k<=1;++k){int nx=x+k,ny=y+j;if(nx<x0||ny<y0||nx>x1||ny>y1)continue;
                    size_t q=size_t(ny)*w+nx;int at=index(nx,ny);if(seen[at]||labels[q]!=parent)continue;
                    float f2=plane.front(pts[q]);if(f2<p_.box_grow||f2>p_.box_max_front)continue;
                    // No geometric bridge across disconnected surfaces.
                    const auto&a=pts[i];const auto&b=pts[q];float dd=(a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);
                    float link=std::min(p_.link_max_m,p_.link_base_m+p_.link_per_m*std::min(a.z,b.z));
                    if(dd>link*link)continue;
                    seen[at]=1;region.push_back(q);
                }
            }
            Target t=describe(pts,region);
            if(edge||strong<2||!boxSize(t))continue;
            float middle=med(depths);if(middle<p_.box_seed||middle>p_.box_max_front)continue;
            // Compact occupied face rather than scattered extrema inflating its size.
            int bx0=w,bx1=0,by0=h,by1=0;for(auto i:region){bx0=std::min(bx0,int(i%w));bx1=std::max(bx1,int(i%w));by0=std::min(by0,int(i/w));by1=std::max(by1,int(i/w));}
            if(region.size()<.60*(bx1-bx0+1)*(by1-by0+1))continue;
            // Refine only after broad candidate validation. Keep a coherent front
            // surface, not the weak transition points connecting it to the wall.
            const float cutoff=std::max(p_.box_seed,middle-.012f);
            std::vector<uint8_t> coreMask(pts.size(),0);
            for(auto i:region)if(plane.front(pts[i])>=cutoff)coreMask[i]=1;
            std::vector<size_t> core,bestCore;
            for(auto seedId:region){
                if(coreMask[seedId]!=1)continue;
                core.clear();core.push_back(seedId);coreMask[seedId]=2;
                for(size_t n=0;n<core.size();++n){int x=core[n]%w,y=core[n]/w;
                    for(int j=-1;j<=1;++j)for(int k=-1;k<=1;++k){
                        int xx=x+k,yy=y+j;if(xx<0||yy<0||xx>=int(w)||yy>=int(h))continue;
                        size_t q=size_t(yy)*w+xx;if(coreMask[q]==1){coreMask[q]=2;core.push_back(q);}
                    }
                }
                if(core.size()>bestCore.size())bestCore=core;
            }
            const auto tight=describe(pts,bestCore);
            if(tight.count<6||tight.width<.06f||tight.height<.035f)continue;
            region=std::move(bestCore);
            proposals.push_back({std::move(region),parent,middle});
        }
        std::sort(proposals.begin(),proposals.end(),[](const Candidate&a,const Candidate&b){return a.ids.size()>b.ids.size();});
        for(const auto&c:proposals){if(out.automatic.split_objects>=8)break;
            bool overlap=false;for(auto i:c.ids){int x=i%w,y=i/w;for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){int xx=x+dx,yy=y+dy;if(xx>=0&&yy>=0&&xx<int(w)&&yy<int(h)&&out.labels[size_t(yy)*w+xx]>original)overlap=true;}}
            if(overlap)continue;
            std::vector<uint8_t> remove(pts.size(),0);for(auto i:c.ids)remove[i]=1;
            std::vector<size_t> rest;for(size_t i=0;i<pts.size();++i)if(out.labels[i]==c.parent&&!remove[i])rest.push_back(i);
            if(rest.size()<p_.min_points)continue;
            auto parent=describe(pts,rest),child=describe(pts,c.ids);parent.id=c.parent;out.targets[c.parent-1]=parent;
            child.id=out.targets.size()+1;child.flags=3;out.targets.push_back(child);for(auto i:c.ids)out.labels[i]=child.id;
            ++out.automatic.components;++out.automatic.accepted;++out.automatic.split_objects;
        }
    }

public:
    explicit Detector(Parameters p={}):p_(p) {
        if(!std::isfinite(p.box_min_width)||!std::isfinite(p.box_max_width)||
           !std::isfinite(p.box_min_height)||!std::isfinite(p.box_max_height)||
           !std::isfinite(p.box_seed)||!std::isfinite(p.box_grow)||!std::isfinite(p.box_max_front)||
           p.box_min_width<=0||p.box_max_width<p.box_min_width||p.box_min_height<=0||
           p.box_max_height<p.box_min_height||p.box_grow<=0||p.box_seed<=p.box_grow||p.box_max_front<=p.box_seed||
           !std::isfinite(p.link_base_m)||!std::isfinite(p.link_per_m)||
           !std::isfinite(p.link_max_m)||!std::isfinite(p.min_long_span_m)||
           !std::isfinite(p.min_second_span_m)||p.link_base_m<=0||p.link_per_m<0||
           p.link_max_m<p.link_base_m||!p.min_points||p.min_long_span_m<0||
           p.min_second_span_m<0) throw std::invalid_argument("Invalid cluster parameters");
    }
    void reset() {} // Stateless: no background or tracking history.
    Result process(unsigned width,unsigned height,const std::vector<Pixel>& pixels) {
        if(!width||!height||static_cast<uint64_t>(width)*height!=pixels.size()||pixels.size()>1000000)
            throw std::invalid_argument("Invalid organised cloud dimensions");
        Result out;out.labels.resize(pixels.size());
        std::vector<uint8_t> seen(pixels.size(),0);
        std::vector<float> radius(pixels.size());
        for(size_t i=0;i<pixels.size();++i) {
            if(!valid(pixels[i])) {seen[i]=1;continue;}
            const auto& p=pixels[i];
            const double range=std::sqrt(double(p.x)*p.x+double(p.y)*p.y+double(p.z)*p.z);
            radius[i]=static_cast<float>(std::min(double(p_.link_max_m),p_.link_base_m+p_.link_per_m*range));
            ++out.automatic.valid;
        }
        std::vector<size_t> queue;queue.reserve(pixels.size());
        for(size_t seed=0;seed<pixels.size();++seed) {
            if(seen[seed])continue;
            ++out.automatic.components;queue.clear();queue.push_back(seed);seen[seed]=1;
            double sx=0,sy=0,sz=0;
            float lo[3]={pixels[seed].x,pixels[seed].y,pixels[seed].z};
            float hi[3]={lo[0],lo[1],lo[2]};double nearest=1e100;
            for(size_t head=0;head<queue.size();++head) {
                const size_t i=queue[head];const auto& a=pixels[i];
                sx+=a.x;sy+=a.y;sz+=a.z;
                float xyz[3]={a.x,a.y,a.z};
                for(int k=0;k<3;++k){lo[k]=std::min(lo[k],xyz[k]);hi[k]=std::max(hi[k],xyz[k]);}
                nearest=std::min(nearest,std::sqrt(double(a.x)*a.x+double(a.y)*a.y+double(a.z)*a.z));
                const int row=static_cast<int>(i/width),col=static_cast<int>(i%width);
                for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
                    if(!dx&&!dy)continue;
                    const int r=row+dy,c=col+dx;
                    if(r<0||c<0||r>=static_cast<int>(height)||c>=static_cast<int>(width))continue;
                    const size_t j=static_cast<size_t>(r)*width+c;if(seen[j])continue;
                    ++out.automatic.edge_checks;
                    const auto& b=pixels[j];
                    const double x=double(a.x)-b.x,y=double(a.y)-b.y,z=double(a.z)-b.z;
                    // Both endpoints must meet their range-dependent threshold.
                    const double limit=std::min(radius[i],radius[j]);
                    if(x*x+y*y+z*z>limit*limit)continue;
                    seen[j]=1;queue.push_back(j);
                }
            }
            std::array<float,3> spans{hi[0]-lo[0],hi[1]-lo[1],hi[2]-lo[2]};
            std::sort(spans.begin(),spans.end());
            if(queue.size()<p_.min_points||spans[2]<p_.min_long_span_m||spans[1]<p_.min_second_span_m) {
                ++out.automatic.rejected;continue;
            }
            Target t;t.id=static_cast<uint32_t>(out.targets.size()+1);t.count=static_cast<uint32_t>(queue.size());
            t.x=static_cast<float>(sx/queue.size());t.y=static_cast<float>(sy/queue.size());t.z=static_cast<float>(sz/queue.size());
            t.width=hi[0]-lo[0];t.height=hi[1]-lo[1];t.extent_z=hi[2]-lo[2];t.nearest=static_cast<float>(nearest);
            out.targets.push_back(t);
            for(auto i:queue)out.labels[i]=t.id;
            out.automatic.labelled+=t.count;++out.automatic.accepted;
        }
        splitBoxes(width,height,pixels,out);
        return out;
    }
};
}
