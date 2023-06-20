#include <iostream>
#include <numeric>

//#include <map_utils/Grid3DROS.h>
#include <map_utils/Grid3D.h>
#include <map_utils/Grid3DBoundingBox.h>
#include <mex_class_wrapper/class_handle.hpp>
#include <mex_conversion_utils/ConversionUtils.h>
#include <Eigen/Geometry>

#include <yaml_utils/YamlUtils.h>

#include "mex.h"

#include <chrono>

namespace m3=map_utils::grid3d;
namespace cu=conversion_utils;
namespace yu=yaml_utils;

typedef m3::CellValue Logodds;

class mystream : public std::streambuf
{
protected:
  virtual std::streamsize xsputn(const char *s, std::streamsize n) { mexPrintf("%.*s", n, s); return n; }
  virtual int overflow(int c=EOF) { if (c != EOF) { mexPrintf("%.1s", &c); } return 1; }
};
class scoped_redirect_cout
{
public:
	scoped_redirect_cout() { old_buf = std::cout.rdbuf(); std::cout.rdbuf(&mout); }
	~scoped_redirect_cout() { std::cout.rdbuf(old_buf); }
private:
	mystream mout;
	std::streambuf *old_buf;
};
static scoped_redirect_cout mycout_redirect;

void
mexFunction(int nlhs, mxArray *plhs[],
            int nrhs, const mxArray *prhs[])
{
  // Get the command string
  char cmd[64];
  if (nrhs < 1 || mxGetString(prhs[0], cmd, sizeof(cmd)))
    mexErrMsgTxt("First input should be a command string less than 64 characters long.");

  if (!strcmp("new", cmd))
  {
    if (nlhs != 1)
    {
      mexErrMsgTxt("New: outputs required: <object_handle>");
      return;
    }

    if (nrhs != 2)
    {
      mexErrMsgTxt("New: inputs required : <\"new\", filename>");
      return;
    }

    char params[512];
    mxGetString(prhs[1], params, sizeof(params));
    std::string filename(params);

    YAML::Node node;
    if (!yu::open(filename, node)) return;

    unsigned int width, height, depth;
    float resolution;

    if (!yu::get("map/width", node, width)) return;
    if (!yu::get("map/height", node, height)) return;
    if (!yu::get("map/depth", node, depth)) return;
    if (!yu::get("map/resolution", node, resolution)) return;

    float origin_x, origin_y, origin_z;
    if (!yu::get("map/origin_x", node, origin_x)) return;
    if (!yu::get("map/origin_y", node, origin_y)) return;
    if (!yu::get("map/origin_z", node, origin_z)) return;

    m3::Point origin(origin_x, origin_y, origin_z);
    plhs[0] = convertPtr2Mat<m3::Grid3D<Logodds>>(new m3::Grid3D<Logodds>(width, height, depth,
                                                        resolution, origin));
    m3::Grid3D<Logodds> *grid = convertMat2Ptr<m3::Grid3D<Logodds>>(plhs[0]);

    float min_clamp, max_clamp;
    if (!yu::get("map/clamping_min", node, min_clamp)) return;
    if (!yu::get("map/clamping_max", node, max_clamp)) return;
    grid->setClampingThresholds(min_clamp, max_clamp);

    float free_threshold, occupancy_threshold;
    if (!yu::get("map/free_threshold", node, free_threshold)) return;
    if (!yu::get("map/occupancy_threshold", node, occupancy_threshold)) return;
    grid->setFreeThreshold(free_threshold);
    grid->setOccupancyThreshold(occupancy_threshold);

    float probability_hit, probability_miss;
    if (!yu::get("map/probability_hit", node, probability_hit)) return;
    if (!yu::get("map/probability_miss", node, probability_miss)) return;
    grid->setLogOddsHit(probability_hit);
    grid->setLogOddsMiss(probability_miss);

    unsigned int block_size;
    if (!yu::get("map/block_size", node, block_size)) return;
    grid->setBlockSize(block_size);

    bool track_changes, lock_at_max_clamp;
    if (!yu::get("map/track_changes", node, track_changes)) return;
    if (!yu::get("map/lock_at_max_clamp", node, lock_at_max_clamp)) return;
    grid->setTrackChanges(track_changes);
    grid->setLockAtMaxClamp(lock_at_max_clamp);

    return;
  }

  //New
  if (!strcmp("new_from_msg", cmd))
  {
    if (nlhs != 1)
    {
      mexErrMsgTxt("NewFromMsg: outputs required: <object_handle>");
      return;
    }

    if (nrhs != 16)
    {
      mexErrMsgTxt("NewFromMsg: inputs required : <\"new\", origin, resolution, log_odds_hit, log_odds_miss, min_clamp, max_clamp, track_changes, occupancy_threshold, free_threshold, block_size, width, height, depth, lock_at_max_clamp, data>");
      return;
    }

    Eigen::Vector3d pt;
    cu::matlabToEigenVec3d(prhs, 1, pt);
    m3::Point origin(pt(0), pt(1), pt(2));

    double resolution = cu::matlabToScalar(prhs, 2);

    float width = cu::matlabToScalar(prhs, 11);
    float height = cu::matlabToScalar(prhs, 12);
    float depth = cu::matlabToScalar(prhs, 13);

    plhs[0] = convertPtr2Mat<m3::Grid3D<Logodds>>(new m3::Grid3D<Logodds>(width, height, depth,
                                                        resolution, origin));

    m3::Grid3D<Logodds> *grid = convertMat2Ptr<m3::Grid3D<Logodds>>(plhs[0]);

    grid->min_clamp = cu::matlabToScalar(prhs, 5);
    grid->max_clamp = cu::matlabToScalar(prhs, 6);
    grid->occupancy_threshold = cu::matlabToScalar(prhs, 8);
    grid->free_threshold = cu::matlabToScalar(prhs, 9);
    grid->log_odds_hit = cu::matlabToScalar(prhs, 3);
    grid->log_odds_miss = cu::matlabToScalar(prhs, 4);
    grid->block_size = cu::matlabToScalar(prhs, 10);
    grid->track_changes = cu::matlabToScalar(prhs, 7);
    grid->lock_at_max_clamp = cu::matlabToScalar(prhs, 14);

    Eigen::MatrixXf logodds;
    cu::matlabToEigen(prhs, 15, logodds);
    grid->data.resize(logodds.rows());
    for (int i = 0; i < logodds.rows(); ++i)
      grid->data[i] = logodds(i,0);

    return;
  }

  if (!strcmp("print_parameters", cmd))
  {

    if (nrhs != 2)
    {
      mexErrMsgTxt("printParameters: inputs required: <\"print_parameters\", object_handle>");
      return;
    }

    m3::Grid3D<Logodds> *grid = convertMat2Ptr<m3::Grid3D<Logodds>>(prhs[1]);

    std::cerr << "Grid3D Parameters: \n"
              << "\twidth: " << grid->width << "\n"
              << "\theight: " << grid->height << "\n"
              << "\tdepth: " << grid->depth << "\n"
              << "\tresolution: " << grid->resolution<< "\n"
              << "\tload_time: " << grid->load_time<< "\n"
              << "\torigin: " << grid->origin.x <<", " << grid->origin.y <<", " << grid->origin.z<< "\n"
              << "\tlog_odds_hit: " << grid->log_odds_hit << "\n"
              << "\tlog_odds_miss: " << grid->log_odds_miss << "\n"
              << "\tfree_threshold: " << grid->free_threshold << "\n"
              << "\toccupancy_threshold: " << grid->occupancy_threshold << "\n"
              << "\tmin_clamp: " << grid->min_clamp << "\n"
              << "\tmax_clamp: " << grid->max_clamp << "\n"
              << "\tblock_size: " << grid->block_size << "\n"
              << "\ttrack_changes: " << grid->track_changes << "\n"
              << "\tlock_at_max_clamp: " << grid->lock_at_max_clamp << "\n"
              << std::endl;
    return;
  }

  if (!strcmp("add_ray", cmd))
  {
    if (nlhs != 0)
    {
      mexErrMsgTxt("addRay: no outputs required");
      return;
    }

    if (nrhs != 5)
    {
      mexErrMsgTxt("addRay: <start, end, max_range>");
      return;
    }

    m3::Grid3D<Logodds> *grid = convertMat2Ptr<m3::Grid3D<Logodds>>(prhs[1]);

    Eigen::Vector3d p;
    cu::matlabToEigenVec3d(prhs, 2, p);
    m3::Point start(p(0), p(1), p(2));

    cu::matlabToEigenVec3d(prhs, 3, p);
    m3::Point end(p(0), p(1), p(2));

    float max_range = cu::matlabToScalar(prhs, 4);
    grid->addRay(start, end, max_range);
    return;
  }

  if (!strcmp("probability", cmd))
  {
    if (nlhs != 1)
    {
      mexErrMsgTxt("Probability: outputs required: <probability>");
      return;
    }

    if (nrhs != 3)
    {
      mexErrMsgTxt("probability: inputs required : <\"probability\", object_handle, point>");
      return;
    }

    Eigen::MatrixXf mat;
    cu::matlabToEigen(prhs, 2, mat);

    if (mat.cols() != 3)
    {
      mexErrMsgTxt("probability: matrix must be nx3");
      return;
    }

    m3::Grid3D<Logodds> *grid = convertMat2Ptr<m3::Grid3D<Logodds>>(prhs[1]);

    std::vector<double> probabilities;
    probabilities.reserve(mat.rows());
    for (int i = 0; i < mat.rows(); ++i)
    {
      m3::Point pt(mat(i,0), mat(i,1), mat(i,2));
      if (!grid->inQ(pt))
        probabilities.push_back(0.5);
      else
      {
	Logodds c = grid->get(pt);
        float p = grid->probability(c.logodds);
        probabilities.push_back(p);
      }
    }
    cu::stdVectorToMATLAB(probabilities, 0, plhs);
    return;
  }

  if (!strcmp("logodds", cmd))
  {
    if (nrhs != 3)
    {
      mexErrMsgTxt("logoods: inputs required: <\"logodds\", object_handle, idx");
      return;
    }

    m3::Grid3D<Logodds> *grid = convertMat2Ptr<m3::Grid3D<Logodds>>(prhs[1]);
    unsigned int idx = cu::matlabToScalar(prhs, 2);
    if (idx < grid->data.size())
    {
      float logodds = grid->data[idx].logodds;
      cu::scalarToMATLAB<float>(logodds, 0, plhs);
    }
    else
      mexErrMsgTxt("idx not in grid");

    return;
  }

  if (!strcmp("get_parameters", cmd))
  {
    if (nrhs != 2)
    {
      mexErrMsgTxt("getParameters: inputs required: <\"get_parameters\", object_handle");
      return;
    }

    m3::Grid3D<Logodds> *grid = convertMat2Ptr<m3::Grid3D<Logodds>>(prhs[1]);
    uint32_t idx = 0;
    Eigen::Matrix<float, 3, 1> origin;
    origin << grid->origin.x, grid->origin.y, grid->origin.z;

    cu::eigenToMATLAB(origin, idx, plhs); idx++;
    cu::scalarToMATLAB<float>(grid->resolution, idx, plhs); idx++;
    cu::scalarToMATLAB<unsigned int>(grid->width, idx, plhs); idx++;
    cu::scalarToMATLAB<unsigned int>(grid->height, idx, plhs); idx++;
    cu::scalarToMATLAB<unsigned int>(grid->depth, idx, plhs); idx++;
    cu::scalarToMATLAB<float>(grid->log_odds_hit, idx, plhs); idx++;
    cu::scalarToMATLAB<float>(grid->log_odds_miss, idx, plhs); idx++;
    cu::scalarToMATLAB<float>(grid->min_clamp, idx, plhs); idx++;
    cu::scalarToMATLAB<float>(grid->max_clamp, idx, plhs); idx++;
    cu::scalarToMATLAB<bool>(grid->track_changes, idx, plhs); idx++;
    cu::scalarToMATLAB<float>(grid->occupancy_threshold, idx, plhs); idx++;
    cu::scalarToMATLAB<float>(grid->free_threshold, idx, plhs); idx++;
    cu::scalarToMATLAB<bool>(grid->track_changes, idx, plhs); idx++;
    cu::scalarToMATLAB<bool>(grid->lock_at_max_clamp, idx, plhs); idx++;

    return;
  }

  if (!strcmp("get_index", cmd))
  {
    if (nrhs != 3)
    {
      mexErrMsgTxt("getIndex: inputs required: <\"get_index\", object_handle, pts (3xN)");
      return;
    }

    uint32_t idx = 1;
    m3::Grid3D<Logodds> *grid = convertMat2Ptr<m3::Grid3D<Logodds>>(prhs[idx]); idx++;

    Eigen::MatrixXf pts;
    cu::matlabToEigen(prhs, idx, pts);
    std::vector<unsigned int> idxs;

    for (uint32_t i = 0; i < pts.cols(); ++i)
    {
      m3::Point p(pts(0,i), pts(1,i), pts(2,i));
      if (!grid->inQ(p))
      {
        std::cerr << "point : " << p.x << ", " << p.y << ", " << p.z << " not found." << std::endl;
        return;
      }

      unsigned int index = grid->getIndex(p);
      idxs.push_back(index);
    }

    cu::stdVectorToMATLAB(idxs, 0, plhs);
    return;
  }

  if (!strcmp("get_probability", cmd))
  {
    if (nrhs != 2)
    {
      mexErrMsgTxt("getProbability: inputs required: <\"get_probability\", object_handle");
      return;
    }

    uint32_t idx = 1;
    m3::Grid3D<Logodds> *grid = convertMat2Ptr<m3::Grid3D<Logodds>>(prhs[idx]); idx++;

    std::vector<float> probabilities;

    for (uint32_t i = 0; i < grid->data.size(); ++i)
    {
      float p = grid->probability(grid->data[i].logodds);
      probabilities.push_back(p);
    }
    cu::stdVectorToMATLAB(probabilities, 0, plhs);
    return;
  }


  if (!strcmp("get_xyz_probability", cmd))
  {
    if (nrhs != 2)
    {
      mexErrMsgTxt("getXYZProbability: inputs required: <\"get_probability\", object_handle");
      return;
    }

    if (nlhs != 2)
    {
      mexErrMsgTxt("getXYZProbability: outputs required: <\"xyz, probs\"");
      return;
    }

    uint32_t idx = 1;
    m3::Grid3D<Logodds> *grid = convertMat2Ptr<m3::Grid3D<Logodds>>(prhs[idx]); idx++;

    std::vector<float> probabilities;

    for (uint32_t i = 0; i < grid->data.size(); ++i)
    {
      float p = grid->probability(grid->data[i].logodds);
      probabilities.push_back(p);
    }

    Eigen::MatrixXf pts = Eigen::MatrixXf::Zero(3, grid->data.size());
    for (unsigned int i = 0; i < grid->data.size(); ++i)
    {
      m3::Point p = grid->getPoint(i);
      pts.col(i) << p.x, p.y, p.z;
    }
    cu::eigenToMATLAB(pts, 0, plhs);
    cu::stdVectorToMATLAB(probabilities, 1, plhs);
    return;
  }

  if (!strcmp("get_point", cmd))
  {
    if (nrhs != 3)
    {
      mexErrMsgTxt("getPoint: inputs required: <\"get_point\", object_handle, idxs>");
      return;
    }

    uint32_t idx = 1;
    std::vector<unsigned int> idxs;

    m3::Grid3D<Logodds> *grid = convertMat2Ptr<m3::Grid3D<Logodds>>(prhs[idx]); idx++;
    cu::matlabToStdVector<unsigned int>(prhs, idx, idxs); idx++;
    Eigen::MatrixXf pts = Eigen::MatrixXf::Zero(3, idxs.size());

    for (uint32_t i = 0; i < idxs.size(); ++i)
    {
      m3::Point pt = grid->getPoint(idxs[i]);
      pts.col(i) << pt.x, pt.y, pt.z;
    }

    cu::eigenToMATLAB(pts, 0, plhs);
    return;
  }

  if (!strcmp("get_ray_points", cmd))
  {
    if (nrhs != 4)
    {
      mexErrMsgTxt("getRayPoints: inputs required: <\"get_ray_points\", object_handle, start, end>");
      return;
    }

    uint32_t idx = 1;
    std::vector<unsigned int> idxs;

    m3::Grid3D<Logodds> *grid = convertMat2Ptr<m3::Grid3D<Logodds>>(prhs[idx]); idx++;
    Eigen::Vector3f start_pt, end_pt;
    cu::matlabToEigenVec3f(prhs, idx, start_pt); idx++;
    cu::matlabToEigenVec3f(prhs, idx, end_pt); idx++;

    m3::Point start(start_pt(0), start_pt(1), start_pt(2));
    m3::Point end(end_pt(0), end_pt(1), end_pt(2));

    std::vector<m3::Cell> raycells;
    bool ret = grid->getRay(start, end, raycells);

    Eigen::Matrix<float, 3, -1> cells(3, raycells.size());
    for (uint32_t i = 0; i < raycells.size(); ++i)
    {
      m3::Point p = grid->getPoint(grid->getIndex(raycells[i]));
      cells.col(i) << p.x, p.y, p.z;
    }
    cu::eigenToMATLAB(cells, 0, plhs);
    cu::scalarToMATLAB(ret, 1, plhs);
    return;
  }

  if (!strcmp("in_q_cell", cmd))
  {
    if (nrhs != 3)
    {
      mexErrMsgTxt("inQCell: inputs required: <\"inQ\", object_handle, 3d_point>");
      return;
    }

    uint32_t idx = 1;
    m3::Grid3D<Logodds> *grid = convertMat2Ptr<m3::Grid3D<Logodds>>(prhs[idx]); idx++;

    Eigen::Vector3f p;
    cu::matlabToEigenVec3f(prhs, idx, p); idx++;
    m3::Cell c;
    c.row = p(0);
    c.col = p(1);
    c.slice = p(2);

    bool inside = grid->inQ(c);
    cu::scalarToMATLAB(inside, 0, plhs);
    return;
  }

  if (!strcmp("w2c", cmd))
  {
    if (nrhs != 3)
    {
      mexErrMsgTxt("w2c: inputs required: <\"w2c\", object_handle, 3d_point>");
      return;
    }

    uint32_t idx = 1;
    m3::Grid3D<Logodds> *grid = convertMat2Ptr<m3::Grid3D<Logodds>>(prhs[idx]); idx++;

    Eigen::Vector3f p;
    cu::matlabToEigenVec3f(prhs, idx, p); idx++;
    m3::Point s(p(0), p(1), p(2));
    m3::Cell c = grid->w2c(s);
    p << c.row, c.col, c.slice;
    cu::eigenToMATLAB(p, 0, plhs);
    return;
  }

  if (!strcmp("in_q_point", cmd))
  {
    if (nrhs != 3)
    {
      mexErrMsgTxt("inQPoint: inputs required: <\"inQ\", object_handle, 3d_point>");
      return;
    }

    uint32_t idx = 1;
    m3::Grid3D<Logodds> *grid = convertMat2Ptr<m3::Grid3D<Logodds>>(prhs[idx]); idx++;

    Eigen::Vector3f p;
    cu::matlabToEigenVec3f(prhs, idx, p); idx++;
    m3::Point s(p(0), p(1), p(2));

    bool inside = grid->inQ(s);
    cu::scalarToMATLAB(inside, 0, plhs);
    return;
  }

  if (!strcmp("reset_change_set", cmd))
  {
    if (nrhs != 2)
    {
      mexErrMsgTxt("resetChangeSet: inputs required: <\"reset_change_set\", object_handle>");
      return;
    }

    uint32_t idx = 1;
    m3::Grid3D<Logodds> *grid = convertMat2Ptr<m3::Grid3D<Logodds>>(prhs[idx]); idx++;
    grid->resetChangeSet();
    return;
  }

  if (!strcmp("get_num_changes", cmd))
  {
    if (nrhs != 2)
    {
      mexErrMsgTxt("getNumChanges: inputs required: <\"get_num_changes\", object_handle>");
      return;
    }

    uint32_t idx = 1;
    m3::Grid3D<Logodds> *grid = convertMat2Ptr<m3::Grid3D<Logodds>>(prhs[idx]); idx++;
    double num_changes = grid->changes.size();
    cu::scalarToMATLAB(num_changes, 0, plhs);
    return;
  }

  //Delete
  if (!strcmp("delete", cmd))
  {
    if (nlhs != 0)
    {
      mexErrMsgTxt("Delete: outputs required: none");
      return;
    }

    if (nrhs != 2)
    {
      mexErrMsgTxt("Delete: inputs required : <\"delete\", objectHandle>");
      return;
    }

    destroyObject<m3::Grid3D<Logodds>>(prhs[1]);

    return;
  }

  mexErrMsgTxt("Command not recognized.");
  return;
}
