#pragma once

#include <smacc/component.h>
#include <move_base_z_client_plugin/components/pose/cp_pose.h>

namespace sm_fetch_two_table_whiskey_pour
{
    namespace cl_perception_system
    {
        struct TableInfo
        {
            std::string associatedCubeColor_ = "";
            cl_move_base_z::Pose *pose_ = nullptr;
            int cubesCounter_ = 0;
        };

        enum class CubeLocation
        {
            ORIGIN_TABLE,
            DESTINY_TABLE
        };

        struct CubeInfo
        {
            std::string color = "";
            TableInfo *dstTableInfo_ = nullptr;
            CubeLocation location_ = CubeLocation::ORIGIN_TABLE;
            cl_move_base_z::Pose *pose_ = nullptr;
        };

        /*
        This component stores the information about the state of the scene.
        It might be updated by other components or behaviors ()
        */
        class CpSceneState : public smacc::ISmaccComponent
        {
        public:
            std::vector<CubeInfo> cubeInfos_;
            std::vector<TableInfo> tablesInfo_;

            CpSceneState(int cubeCount, int tableCount)
            {
                cubeInfos_.resize(cubeCount);
                tablesInfo_.resize(tableCount);
            }

            template <typename TOrthogonal, typename TSourceObject>
            void onOrthogonalAllocation()
            {
                // create table "poses track components"
                std::vector<std::string> availableColors = {"none", "yellow", "white", "purple", "red", "green"};
                for (int i = 0; i < tablesInfo_.size(); i++)
                {
                    auto tablename = "table_" + std::to_string(i + 1);
                    this->tablesInfo_[i].pose_ = this->createSiblingNamedComponent<cl_move_base_z::Pose, TSourceObject, TOrthogonal>(tablename, tablename, "map");
                    this->tablesInfo_[i].associatedCubeColor_ = availableColors[i];
                }

                // create cube "poses track components"
                for (int i = 0; i < cubeInfos_.size(); i++)
                {
                    auto cubename = "cube_" + std::to_string(i + 1);
                    this->cubeInfos_[i].pose_ = this->createSiblingNamedComponent<cl_move_base_z::Pose, TSourceObject, TOrthogonal>(cubename, cubename, "map");
                }
            }
        };
    } // namespace cl_perception_system
};    // namespace sm_fetch_two_table_whiskey_pour
