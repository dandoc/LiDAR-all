#pragma once
#include "target_detector.h"
#include <limits>

namespace Watch {
// Lightweight alpha-beta tracking in SENSOR coordinates. Relative velocity
// includes sensor motion; this is not world odometry or absolute object speed.
class BoxTracker {
    struct Track {
        Target t;
        std::array<float,3> pos{},velocity{};
        double last_seen=0;
        unsigned streak=1;
        bool confirmed=false;
    };
    std::vector<Track> tracks_;
    uint32_t next_=0x80000001u;
    double previous_=-1;
public:
    void reset(){tracks_.clear();previous_=-1;} // Never reuse IDs in a process.
    void update(Result& result,double now){
        if(!std::isfinite(now)||now<0)throw std::invalid_argument("Invalid tracking time");
        if(previous_>=0&&now<=previous_)throw std::invalid_argument("Non-increasing tracking time");
        double dt=previous_<0?0:now-previous_;previous_=now;
        tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[&](const Track&t){
            return now-t.last_seen>(t.confirmed?.8:.4);
        }),tracks_.end());
        for(auto&tr:tracks_)for(int k=0;k<3;++k)tr.pos[k]+=tr.velocity[k]*float(dt);
        std::vector<size_t> boxes;
        for(size_t i=0;i<result.targets.size();++i)if(result.targets[i].flags&2)boxes.push_back(i);
        struct Edge{size_t track,box;float cost;};std::vector<Edge> edges;
        for(size_t i=0;i<tracks_.size();++i)for(size_t j=0;j<boxes.size();++j){
            const auto&tr=tracks_[i];const auto&t=result.targets[boxes[j]];
            float dx=t.x-tr.pos[0],dy=t.y-tr.pos[1],dz=t.z-tr.pos[2];
            float distance=std::sqrt(dx*dx+dy*dy+dz*dz);
            // At 5 Hz allow 28 cm prediction error; cap ambiguous long jumps.
            float gate=std::min(.45f,.12f+.8f*float(now-tr.last_seen));
            float rw=std::max(t.width,tr.t.width)/std::max(.02f,std::min(t.width,tr.t.width));
            float rh=std::max(t.height,tr.t.height)/std::max(.02f,std::min(t.height,tr.t.height));
            if(distance<=gate&&rw<2.5f&&rh<2.5f)
                edges.push_back({i,j,distance+.04f*(std::abs(std::log(rw))+std::abs(std::log(rh)))});
        }
        std::sort(edges.begin(),edges.end(),[](const Edge&a,const Edge&b){return a.cost<b.cost;});
        std::vector<int> matched(boxes.size(),-1);std::vector<bool> used(tracks_.size(),false);
        for(const auto&e:edges){
            if(used[e.track]||matched[e.box]>=0)continue;
            bool ambiguous=false;
            for(const auto&other:edges)if((other.track==e.track&&other.box!=e.box)||
                                         (other.box==e.box&&other.track!=e.track)){
                if(std::abs(other.cost-e.cost)<.03f){ambiguous=true;break;}
            }
            if(ambiguous)continue; // Prefer an unconfirmed new ID to an ID swap.
            used[e.track]=true;matched[e.box]=int(e.track);
        }
        std::vector<std::pair<uint32_t,uint32_t>> remap;
        for(size_t j=0;j<boxes.size();++j){
            auto&t=result.targets[boxes[j]];uint32_t old=t.id;
            if(matched[j]>=0){
                auto&tr=tracks_[matched[j]];float observation[3]={t.x,t.y,t.z};
                for(int k=0;k<3;++k){float residual=observation[k]-tr.pos[k];
                    tr.pos[k]+=.8f*residual;
                    tr.velocity[k]=std::max(-1.5f,std::min(1.5f,tr.velocity[k]+.15f*residual/float(std::max(.02,dt))));
                }
                ++tr.streak;tr.confirmed=tr.confirmed||tr.streak>=3;
                t.id=tr.t.id;t.age=tr.t.age+1;t.is_new=false;
                t.flags=tr.confirmed?7:3;tr.last_seen=now;tr.t=t;
                // Measured rows remain actual centroids. Filter is for association/prediction.
            }else{
                if(tracks_.size()>=16){t.flags=1;continue;}
                if(next_==std::numeric_limits<uint32_t>::max())throw std::overflow_error("Track ID exhausted");
                t.id=next_++;t.age=1;t.is_new=true;t.flags=3;
                Track tr;tr.t=t;tr.pos={t.x,t.y,t.z};tr.last_seen=now;
                tracks_.push_back(tr);used.push_back(true);
            }
            remap.push_back({old,t.id});
        }
        for(auto&label:result.labels)for(const auto&pair:remap)if(label==pair.first){label=pair.second;break;}
        for(size_t i=0;i<tracks_.size();++i)if(!used[i]){
            auto&tr=tracks_[i];tr.streak=0;
            if(!tr.confirmed)continue;
            auto t=tr.t;t.x=tr.pos[0];t.y=tr.pos[1];t.z=tr.pos[2];
            if(t.z<=0)continue;
            t.count=0;t.flags=15;t.is_new=false;t.nearest=0;
            result.targets.push_back(t); // No point labels invented for missing observations.
        }
    }
};
}
