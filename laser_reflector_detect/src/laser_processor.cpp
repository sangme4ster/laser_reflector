#include <../include/ros_qt5/laser_processor.h>
#include <stdexcept>
#include <list>
#include <set>
namespace laser_processor
{
Sample* Sample::Extract(int ind, int inensity, const sensor_msgs::LaserScan& scan)
{
  Sample* s = new Sample();

  s->index = ind;
  s->range = scan.ranges[ind];
  s->intensity = scan.intensities[ind];
  // ROS_INFO("ranges[%d] = %f,intensities[%d] = %f",ind,scan.ranges[ind],ind,scan.intensities[ind]);
  s->x = cos(scan.angle_min + ind * scan.angle_increment) * s->range;
  s->y = sin(scan.angle_min + ind * scan.angle_increment) * s->range;
  if (     s->range > scan.range_min
        && s->range < scan.range_max
        && s->intensity > inensity
           )
    return s;
  else
  {
    delete s;
    return NULL;
  }
}

void SampleSet::clear()
{
  for (SampleSet::iterator i = begin();
       i != end();
       i++)
  {
    delete(*i);
  }
  set<Sample*, CompareSample>::clear();
}


tf::Point SampleSet::center()
{
  float x_mean = 0.0;
  float y_mean = 0.0;
  for (iterator i = begin();
       i != end();
       i++)

  {
    x_mean += ((*i)->x) / size();
    y_mean += ((*i)->y) / size();
  }

  return tf::Point(x_mean, y_mean, 0.0);
}



ScanProcessor::ScanProcessor(const sensor_msgs::LaserScan& scan, ScanMask& mask_, float mask_threshold)
{
  
}

ScanProcessor::~ScanProcessor()
{
  for (std::list<SampleSet*>::iterator c = clusters_.begin();
       c != clusters_.end();
       c++)
    delete(*c);
}


};  // namespace laser_processor
