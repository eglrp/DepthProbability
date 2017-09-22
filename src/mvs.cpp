//
// Created by jianping on 17-9-21.
//

#include "mvs.h"

bool mvs::estimateDepthMap(int id)
{
    _scene->pdata = std::make_shared<MVS::DenseDepthMapData>(*_scene);
    MVS::DenseDepthMapData& data = *_scene->pdata;
    // maps global view indices to our list of views to be processed
    IndexArr imagesMap;
    // prepare images for dense reconstruction (load if needed)
    {
        LOG(INFO)<<"LOAD IMAGES";
        TD_TIMER_START();
        data.images.Reserve(_scene->images.GetSize());
        imagesMap.Resize(_scene->images.GetSize());


        FOREACH(idxImage, _scene->images) {
            // skip invalid, uncalibrated or discarded images
            MVS::Image& imageData = _scene->images[idxImage];
            if (!imageData.IsValid()) {

                imagesMap[idxImage] = NO_ID;
                continue;
            }
            // map image index
            {
                imagesMap[idxImage] = (uint32_t)data.images.GetSize();
                data.images.Insert(idxImage);
            }
            // reload image at the appropriate resolution
            const unsigned nMaxResolution(imageData.RecomputeMaxResolution(MVS::OPTDENSE::nResolutionLevel, MVS::OPTDENSE::nMinResolution));
            if (!imageData.ReloadImage(nMaxResolution)) {

                return false;
            }
            imageData.UpdateCamera(_scene->platforms);
            // print image camera
            LOG(INFO)<<"K"<<idxImage<<"\n"<<cvMat2String(imageData.camera.K).c_str();
            LOG(INFO)<<"R"<<idxImage<<"\n"<<cvMat2String(imageData.camera.R).c_str();
            LOG(INFO)<<"C"<<idxImage<<"\n"<<cvMat2String(imageData.camera.C).c_str();
        }

        if (data.images.IsEmpty()) {
            LOG(INFO)<<("error: preparing images for dense reconstruction failed (errors loading images)");
            return false;
        }
        LOG(INFO)<<"Preparing images for dense reconstruction completed: "<<
                _scene->images.GetSize()<<" images";
    }

// select images to be used for dense reconstruction
    {
        LOG(INFO)<<"select views";
        TD_TIMER_START();
        // for each image, find all useful neighbor views
        IndexArr invalidIDs;

        FOREACH(idx, data.images) {
            const uint32_t idxImage(data.images[idx]);
            ASSERT(imagesMap[idxImage] != NO_ID);

            MVS::DepthData& depthData(data.detphMaps.arrDepthData[idxImage]);
            if (!data.detphMaps.SelectViews(depthData)) {
                invalidIDs.InsertSort(idx);
            }
        }
        RFOREACH(i, invalidIDs) {
            const uint32_t idx(invalidIDs[i]);
            imagesMap[data.images.Last()] = idx;
            imagesMap[data.images[idx]] = NO_ID;
            data.images.RemoveAt(idx);
        }
        // globally select a target view for each reference image
        if (MVS::OPTDENSE::nNumViews == 1 && !data.detphMaps.SelectViews(data.images, imagesMap, data.neighborsMap)) {
            LOG(INFO)<<"error: no valid images to be dense reconstructed";
            return false;
        }
        ASSERT(!data.images.IsEmpty());
        LOG(INFO)<<"Selecting images for dense reconstruction completed: "<< data.images.GetSize()<<" images";
    }

    /////
    LOG(INFO)<<"select views to reconstruct the depth-map for this image";
    {
        // select views to reconstruct the depth-map for this image
        const uint32_t idx = data.images[id];
        MVS::DepthData &depthData(data.detphMaps.arrDepthData[idx]);
        // init images pair: reference image and the best neighbor view
        ASSERT(data.neighborsMap.IsEmpty() || data.neighborsMap[evtImage.idxImage] != NO_ID);
        if (!data.detphMaps.InitViews(depthData, data.neighborsMap.IsEmpty() ? NO_ID : data.neighborsMap[id],
                                      MVS::OPTDENSE::nNumViews)) {
            LOG(INFO) <<"fail to select views for this image";
            return false;
        }
        LOG(INFO)<<"estimate depth-map for this image";
        data.detphMaps.EstimateDepthMap(data.images[id]);
    }

    {
        const uint32_t idx = data.images[id];
        MVS::DepthData &depthData(data.detphMaps.arrDepthData[idx]);
        // apply filters
        if (MVS::OPTDENSE::nOptimize & (MVS::OPTDENSE::REMOVE_SPECKLES)) {
            LOG(INFO)<<"RemoveSmallSegments for this image";
            data.detphMaps.RemoveSmallSegments(depthData);
        }
        if (MVS::OPTDENSE::nOptimize & (MVS::OPTDENSE::FILL_GAPS)) {
            LOG(INFO)<<"GapInterpolation for this image";
            data.detphMaps.GapInterpolation(depthData);
        }
    }
    LOG(INFO)<<"save depth-map for this image";
    {
        const uint32_t idx = data.images[id];
        MVS::DepthData &depthData(data.detphMaps.arrDepthData[idx]);
#if TD_VERBOSE != TD_VERBOSE_OFF
        // save depth map as image
        if (g_nVerbosityLevel > 2) {
            //MVS::ExportDepthMap(ComposeDepthFilePath(idx, "png"), depthData.depthMap);
            //MVS::ExportConfidenceMap(ComposeDepthFilePath(idx, "conf.png"), depthData.confMap);
            ExportPointCloud(ComposeDepthFilePath(idx, "ply"), *depthData.images.First().pImageData, depthData.depthMap,
                             depthData.normalMap);

            //MVS::ExportNormalMap(ComposeDepthFilePath(idx, "normal.png"), depthData.normalMap);
            //depthData.confMap.Save(ComposeDepthFilePath(idx, "conf.pfm"));
            //depthData.depthMap.Save(ComposeDepthFilePath(idx, "depth.pfm"));
            //depthData.normalMap.Save(ComposeDepthFilePath(idx, "norm.pfm"));

            //MVS::SaveDepthMap(ComposeDepthFilePath(idx, "dmap"),depthData.depthMap);
            //MVS::SaveConfidenceMap(ComposeDepthFilePath(idx, "cmap"),depthData.confMap);
            //MVS::SaveNormalMap(ComposeDepthFilePath(idx, "nmap"),depthData.confMap);
        }
#endif
        // save compute depth-map for this image
        depthData.Save(ComposeDepthFilePath(idx, "dmap"));
        depthData.ReleaseImages();
        depthData.Release();
    }
        /////
    return true;
}


