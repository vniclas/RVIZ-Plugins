#pragma once

#include <sensor_msgs/PointCloud2.h>

#include <rviz/default_plugin/point_cloud_transformer.h>
#include <rviz/default_plugin/point_cloud_transformers.h>

namespace rviz
{

    class SemanticPCTransformer : public PointCloudTransformer
    {
        Q_OBJECT
    public:
        uint8_t supports(const sensor_msgs::PointCloud2ConstPtr &cloud) override;
        bool transform(const sensor_msgs::PointCloud2ConstPtr &cloud,
                       uint32_t mask,
                       const Ogre::Matrix4 &transform,
                       V_PointCloudPoint &points_out) override;
        uint8_t score(const sensor_msgs::PointCloud2ConstPtr &cloud) override;
        void createProperties(Property *parent_property, uint32_t mask, QList<Property *> &out_props) override;
        void updateChannels(const sensor_msgs::PointCloud2ConstPtr &cloud);

    private:
        V_string available_channels_;

        EditableEnumProperty *channel_name_property_;
        EditableEnumProperty *semantic_style_property_;
    };

} // namespace rviz
