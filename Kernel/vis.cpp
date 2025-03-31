#include "vis.h"

#include <random>

#include <pcl/visualization/pcl_visualizer.h>

void kernel::vis::show_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer viewer;
    viewer.addPointCloud(cloud);
    viewer.spin();
}

void kernel::vis::show_cloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
    auto xyz = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*cloud, *xyz);
    pcl::copyPointCloud<pcl::PointNormal, pcl::Normal>(*cloud, *normals);
    show_cloud(xyz, normals);
}

void kernel::vis::show_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer viewer;
    viewer.addPointCloud(cloud);
    viewer.spin();
}

void kernel::vis::show_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::visualization::PCLVisualizer viewer;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> points_color(xyz, 0, 255, 0);
    viewer.addPointCloud(xyz, points_color, "points");
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(xyz, normals, 10, 0.25f, "normals");
    viewer.spin();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr kernel::vis::get_colored_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, const std::vector<pcl::PointIndices>& clusters_indices)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

    if (!clusters_indices.empty())
    {
        colored_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

        std::default_random_engine random_engine;
        random_engine.seed(time(nullptr));

        std::vector<unsigned char> colors;
        for (int i = 0; i < clusters_indices.size(); ++i)
        {
            colors.push_back(static_cast<unsigned char>(random_engine() % 256));
            colors.push_back(static_cast<unsigned char>(random_engine() % 256));
            colors.push_back(static_cast<unsigned char>(random_engine() % 256));
        }

        colored_cloud->width = xyz->width;
        colored_cloud->height = xyz->height;
        colored_cloud->is_dense = xyz->is_dense;
        for (const auto& p : xyz->points)
        {
            colored_cloud->points.emplace_back(p.x, p.y, p.z, 255, 255, 255);
        }

        int next_color = 0;
        for (const auto& cluster_indices : clusters_indices)
        {
            for (const auto& index : cluster_indices.indices)
            {
                colored_cloud->at(index).r = colors[3 * next_color];
                colored_cloud->at(index).g = colors[3 * next_color + 1];
                colored_cloud->at(index).b = colors[3 * next_color + 2];
            }
            next_color++;
        }
    }

    return colored_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr kernel::vis::get_colored_cloud(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

    if (!clouds.empty()) {
        colored_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

        std::default_random_engine random_engine;
        random_engine.seed(time(nullptr));

        std::vector<unsigned char> colors;
        for (int i = 0; i < clouds.size(); ++i)
        {
            colors.push_back(static_cast<unsigned char>(random_engine() % 256));
            colors.push_back(static_cast<unsigned char>(random_engine() % 256));
            colors.push_back(static_cast<unsigned char>(random_engine() % 256));
        }

        for (int i = 0; i < clouds.size(); ++i) {
            for (const auto& p : clouds.at(i)->points) {
                colored_cloud->points.emplace_back(p.x, p.y, p.z, colors[3 * i], colors[3 * i + 1], colors[3 * i + 2]);
            }
        }
    }

    return colored_cloud;
}
