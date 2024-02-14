#include <OgreColourValue.h>
#include <OgreMatrix4.h>
#include <OgreVector3.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/validate_floats.h>

#include "rviz_plugins/point_cloud_transformers.h"

namespace rviz
{
    static void getSemanticColor(int value, Ogre::ColourValue &color, const std::string &semantic_style)
    {
        if (semantic_style == "Carla")
        {
            // https://carla.readthedocs.io/en/latest/ref_sensors/#semantic-segmentation-camera

            switch (value)
            {
            case 0: // unlabeled
                color.r = 0, color.g = 0, color.b = 0;
                break;
            case 1: // building
                color.r = 70, color.g = 70, color.b = 70;
                break;
            case 2: // fence
                color.r = 190, color.g = 153, color.b = 153;
                break;
            case 3: // other
                color.r = 250, color.g = 170, color.b = 160;
                break;
            case 4: // pedestrian
                color.r = 220, color.g = 20, color.b = 60;
                break;
            case 5: // pole
                color.r = 153, color.g = 153, color.b = 153;
                break;
            case 6: // road line
                color.r = 153, color.g = 153, color.b = 153;
                break;
            case 7: // road
                color.r = 128, color.g = 64, color.b = 128;
                break;
            case 8: // sidewalk
                color[0] = 244, color[1] = 35, color[2] = 232;
                break;
            case 9: // vegetation
                color.r = 107, color.g = 142, color.b = 35;
                break;
            case 10: // car
                color.r = 0, color.g = 0, color.b = 142;
                break;
            case 11: // wall
                color.r = 102, color.g = 102, color.b = 156;
                break;
            case 12: // traffic sign
                color.r = 220, color.g = 220, color.b = 0;
                break;
            case 13: // sky
                color.r = 70, color.g = 130, color.b = 180;
                break;
            case 14: // ground
                color.r = 18, color.g = 0, color.b = 81;
                break;
            case 15: // bridge
                color.r = 150, color.g = 100, color.b = 100;
                break;
            case 16: // rail track
                color.r = 230, color.g = 150, color.b = 140;
                break;
            case 17: // guard rail
                color.r = 180, color.g = 165, color.b = 180;
                break;
            case 18: // traffic light
                color.r = 250, color.g = 170, color.b = 30;
                break;
            case 19: // static
                color.r = 110, color.g = 190, color.b = 160;
                break;
            case 20: // dynamic
                color.r = 170, color.g = 120, color.b = 50;
                break;
            case 21: // water
                color.r = 45, color.g = 60, color.b = 150;
                break;
            case 22: // terrain
                color.r = 145, color.g = 170, color.b = 100;
                break;
            default:
                std::cout << "Found an unknown semantic tag: " << value << std::endl;
                color[0] = 0, color[1] = 0, color[2] = 0;
            }
        }
        else
        {
            switch (value)
            {
            case 0: // unlabeled
                color.r = 0, color.g = 0, color.b = 0;
                break;
            case 1: // building
                color.r = 70, color.g = 70, color.b = 70;
                break;
            case 2: // fence
                color.r = 190, color.g = 153, color.b = 153;
                break;
            case 3: // pedestrian
                color.r = 220, color.g = 20, color.b = 60;
                break;
            case 4: // pole
                color.r = 153, color.g = 153, color.b = 153;
                break;
            case 5: // road
                color.r = 128, color.g = 64, color.b = 128;
                break;
            case 6: // sidewalk
                color[0] = 244, color[1] = 35, color[2] = 232;
                break;
            case 7: // vegetation
                color.r = 107, color.g = 142, color.b = 35;
                break;
            case 8: // vehicle
                color.r = 0, color.g = 0, color.b = 142;
                break;
            case 9: // traffic sign
                color.r = 220, color.g = 220, color.b = 0;
                break;
            case 10: // ground
                color.r = 18, color.g = 0, color.b = 81;
                break;
            case 11: // traffic light
                color.r = 250, color.g = 170, color.b = 30;
                break;
            case 12: // static
                color.r = 110, color.g = 190, color.b = 160;
                break;
            default:
                std::cout << "Found an unknown semantic tag: " << value << std::endl;
                color[0] = 0, color[1] = 0, color[2] = 0;
            }
        }

        color.r /= 255.;
        color.g /= 255.;
        color.b /= 255.;
    }

    uint8_t SemanticPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
        updateChannels(cloud);
        return Support_Color;
    }

    uint8_t SemanticPCTransformer::score(const sensor_msgs::PointCloud2ConstPtr & /*cloud*/)
    {
        return 255;
    }

    bool SemanticPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr &cloud,
                                          uint32_t mask,
                                          const Ogre::Matrix4 & /*transform*/,
                                          V_PointCloudPoint &points_out)
    {
        if (!(mask & Support_Color))
        {
            return false;
        }

        int32_t index = findChannelIndex(cloud, channel_name_property_->getStdString());

        if (index == -1)
        {
            if (channel_name_property_->getStdString() == "intensity")
            {
                index = findChannelIndex(cloud, "intensities");
                if (index == -1)
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }

        std::string semantic_style = semantic_style_property_->getStdString();

        const uint32_t offset = cloud->fields[index].offset;
        const uint8_t type = cloud->fields[index].datatype;
        const uint32_t point_step = cloud->point_step;
        const uint32_t num_points = cloud->width * cloud->height;

        for (uint32_t i = 0; i < num_points; ++i)
        {
            float val = valueFromCloud<float>(cloud, offset, type, point_step, i);
            int value = static_cast<int>(val);
            getSemanticColor(value, points_out[i].color, semantic_style);
        }

        return true;
    }

    void SemanticPCTransformer::createProperties(Property *parent_property,
                                                 uint32_t mask,
                                                 QList<Property *> &out_props)
    {
        if (mask & Support_Color)
        {
            channel_name_property_ =
                new EditableEnumProperty("Channel Name", "intensity",
                                         "Select the channel that stores the semantic labels", parent_property,
                                         SIGNAL(needRetransform()), this);

            out_props.push_back(channel_name_property_);

            semantic_style_property_ =
                new EditableEnumProperty("Semantic Style", "Carla",
                                         "Select the style used for the semantic labels", parent_property,
                                         SIGNAL(needRetransform()), this);
            semantic_style_property_->addOptionStd("Carla");
            semantic_style_property_->addOptionStd("Custom");
            out_props.push_back(semantic_style_property_);
        }
    }

    void SemanticPCTransformer::updateChannels(const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
        V_string channels;
        for (size_t i = 0; i < cloud->fields.size(); ++i)
        {
            channels.push_back(cloud->fields[i].name);
        }
        std::sort(channels.begin(), channels.end());

        if (channels != available_channels_)
        {
            channel_name_property_->clearOptions();
            for (V_string::const_iterator it = channels.begin(); it != channels.end(); ++it)
            {
                const std::string &channel = *it;
                if (channel.empty())
                {
                    continue;
                }
                channel_name_property_->addOptionStd(channel);
            }
            available_channels_ = channels;
        }
    }

} // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::SemanticPCTransformer, rviz::PointCloudTransformer)
