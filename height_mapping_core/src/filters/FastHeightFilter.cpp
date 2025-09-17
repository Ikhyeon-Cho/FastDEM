#include "height_mapping_core/filters/FastHeightFilter.h"

namespace height_mapping {

FastHeightFilter::FastHeightFilter(double min_height, double max_height)
    : minZ_(min_height), maxZ_(max_height) {}

template <typename PointT>
void FastHeightFilter::filter(const typename pcl::PointCloud<PointT>::Ptr &input,
                              typename pcl::PointCloud<PointT>::Ptr &output) {
  output->clear();
  output->reserve(input->size());

  for (const auto &point : input->points) {
    if (point.z >= minZ_ && point.z <= maxZ_) {
      output->points.push_back(point);
    }
  }
  output->header = input->header;
}

// Explicit instantiation for the types we use
template void FastHeightFilter::filter<Laser>(const pcl::PointCloud<Laser>::Ptr &input,
                                              pcl::PointCloud<Laser>::Ptr &output);

template void FastHeightFilter::filter<Color>(const pcl::PointCloud<Color>::Ptr &input,
                                              pcl::PointCloud<Color>::Ptr &output);

} // namespace height_mapping