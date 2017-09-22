//
// Created by jianping on 17-9-21.
//

#ifndef DEPTHPROBABILITY_MVS_H
#define DEPTHPROBABILITY_MVS_H

#include <OpenMVS/MVS.h>
#include <glog/logging.h>
#include <memory>
#include <boost/python.hpp>
class mvs {
public:
    mvs(int thread)
    {
        _scene = std::make_shared<MVS::Scene>(thread);
        MVS::OPTDENSE::init();
        //const bool bValidConfig(OPTDENSE::oConfig.Load(OPT::strDenseConfigFileName));
        MVS::OPTDENSE::update();
    }
    bool readScene(const std::string& path)
    {
        if(_scene->Load(path))
        {
            LOG(INFO)<<"Scene image size: "<<_scene->images.size();
            LOG(INFO)<<"Scene point size: "<<_scene->pointcloud.points.size();
            return true;
        } else{
            return false;
        }

    }
    boost::python::list getImage(const int id)
    {
        boost::python::list data;
        LOG(INFO)<<"get image :"<<_scene->images[id].name;
        _scene->images[id].ReloadImage(0,true);

        Image8U3& image = _scene->images[id].image;
        int width = image.cols;
        int height = image.rows;
        data.append(width);
        data.append(height);
        for (int j = 0; j < image.rows*image.cols*3; ++j) {
            data.append(image.data[j]);
        }
        return data;
    }

    bool estimateDepthMap(int id);

    boost::python::list getDepth(const int id)
    {
        boost::python::list data;
        MVS::DepthData depthData;
        depthData.images.resize(1);
        depthData.Load(ComposeDepthFilePath(id, "dmap"));
        const uint32_t idx = _scene->pdata->images[id];
        int width = depthData.depthMap.cols;
        int height = depthData.depthMap.rows;
        data.append(width);
        data.append(height);
        for (int j = 0; j < width*height; ++j) {
            data.append(depthData.depthMap[j]);
        }
        return data;
    }


private:
    std::shared_ptr<MVS::Scene> _scene;
};


#endif //DEPTHPROBABILITY_MVS_H
