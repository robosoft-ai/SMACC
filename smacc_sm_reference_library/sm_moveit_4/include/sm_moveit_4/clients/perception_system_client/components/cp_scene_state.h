#pragma once

#include <smacc/component.h>

namespace sm_moveit_4
{
    namespace cl_perception_system
    {
        struct TableInfo
        {
            std::string targetColor_;
        };

        enum CubeLocation
        {
            ORIGIN_TABLE,
            DESTINY_TABLE
        };

        struct CubeInfo
        {
            std::string color;
            const TableInfo *dstTableInfo_;
            CubeLocation location_;
        };

        /*
        This component stores the information about the state of the scene.
        It might be updated by other components or behaviors ()
        */
        class CpSceneState : public smacc::ISmaccComponent
        {
            public:
                // cubes state, only mutated at the begining of the scene
                std::vector<CubeInfo> cubeInfos_;

                const std::vector<TableInfo> tablesInfo_ = {{"yellow"}, {"white"}, {"purple"}, {"none"}, {"red"}, {"green"}};

                // only mutated at the begining of the scene, does not need thread-safety, pose components are thread safe
                std::vector<cl_move_base_z::Pose *> tablePoses_;
        };
    }
};