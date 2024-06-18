/*
 * @description: 
 * @param : 
 * @return: 
 */
#ifndef _TYPE_H_
#define _TYPE_H_
#include <Eigen/Core>
#include <vector>
#include <assert.h>
#include <opencv2/highgui.hpp>
#include <list>
#include <set>
#include <plane_detection/quatree_node.h>
#include <iostream>
#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plane_detection/point_type.h>
using namespace std;
// [456.67578125, 0.0, 310.76171875, 0.0, 457.3671875, 246.896484375, 0.0, 0.0, 1.0]
const double kFx = 456.67578125;
const double kFy = 457.3671875;
const double kCx = 310.76171875;
const double kCy = 246.896484375;
const int kDepthWidth = 640;
const int kDepthHeight = 480;

// const double kFx = 918.8240966796875;
// const double kFy = 917.1801147460938;
// const double kCx = 664.4627685546875;
// const double kCy = 375.1426696777344;
// const int kDepthWidth = 1280;
// const int kDepthHeight = 720;
const int kNeighborRange = 5; // boundary pixels' neighbor range
// const int kScaleFactor = 5; // scale coordinate unit in mm
const int kScaleFactor = 1000; // scale coordinate unit in mm
const float kInfVal = 1000000; // an infinite large value used in MRF optimization

struct orginazed_points
{
  public:
  size_t width, height;
  // 写成一个vector会不会效率更高
  std::vector<std::vector<Eigen::Vector3f>> points;
  
  void initial(cv::Mat & depth_image)
  {
    width = depth_image.cols; height = depth_image.rows;
    // 先将长方形的点转成正方形
    points.reserve(height);
    for (size_t i = 0; i < height; i++)
    {
      std::vector<Eigen::Vector3f> i_row;
      i_row.reserve(width);
      for (size_t j = 0; j < width; j++)
      {
        // 没有使用迭代器
        float z = (float)(depth_image.at<unsigned short>(i, j)) / kScaleFactor;
        if (std::isnan(z))
        {
          i_row.emplace_back(Eigen::Vector3f(0, 0, z));
          continue;
        }
        float x = ((float)j - kCx) * z / kFx;
        float y = ((float)i - kCy) * z / kFy;
        i_row.emplace_back(Eigen::Vector3f(x, y, z)); 
      }
      points.emplace_back(i_row);
    }
  }
  
  void initialByPCL(pcl::PointCloud<pcl::PointXYZ> & pc)
  {
    width = pc.width; height = pc.height;
    std::cout<<"width: "<<width<<" height: "<<height<<std::endl;
    points.reserve(height);
    for (size_t i = 0; i < height; i++)
    {
      std::vector<Eigen::Vector3f> i_row;
      i_row.reserve(width);
      pcl::PointCloud<pcl::PointXYZ>::iterator start_iter = pc.begin() + i * width;
      pcl::PointCloud<pcl::PointXYZ>::iterator end_iter = pc.begin() + (i+1) * width;
      while (start_iter != end_iter)
      {
        i_row.emplace_back(Eigen::Vector3f(start_iter->x, start_iter->y, start_iter->z));
        start_iter++;
      }
      // {}只表示了其内部的两个数字，并没有表示中间所有的
      // for (auto & iter_point:std::range{pc.begin() + i * width, pc.begin() + (i + 1) * width})
      // {
      //   i_row.emplace_back(Eigen::Vector3f(iter_point->x, iter_point->y, iter_point->z));
      // }
      // std::cout<<i_row.size()<<std::endl;
      assert(i_row.size() == width);
      points.emplace_back(i_row);
    }
  }
  
  //先写在这，看后续是不是需要换成更高效的方式, 需要记录点的像素坐标
  /**
   * @description: get points from raw points
   * @param {size_t} start_rows: left and top point at a rect, the point start row index
   * @param {size_t} start_cols: left and top point at a rect, the point start col index
   * @param {size_t} width: rect width, it is on col direct
   * @param {size_t} height: rect height, it is on row direct
   * @return {*} valid points in rect
   */  
  IndexPoints getRectPoint(size_t start_rows, size_t start_cols, size_t width_, size_t height_)
  {
    
    assert(start_rows < height);
    assert(start_cols < width);
    if (start_rows + height_ > height)
    {
      height_ = height - start_rows;
    }
    if (start_cols + width_ > width)
    {
      width_ = width - start_cols;
    }
    // LOG(INFO)<<"start_cols: "<<start_cols<<" width_: "<<width_<<" start_rows: "<<start_rows<<" height_: "<<height_<<endl;
    // LOG(INFO)<<start_rows<<" "<<height_<<" "<<height<<endl;
    assert(start_rows + height_ <= height);
    // LOG(INFO)<<start_cols<<" "<<width_<<" "<<width<<endl;
    assert(start_cols + width_ <= width);

    // std::cout<<"start_rows: "<<start_rows<<endl;
    // std::cout<<"start_cols: "<<start_cols<<endl;
    // std::cout<<"width_: "<<width_<<endl;
    // std::cout<<"height_: "<<height_<<endl;

    IndexPoints return_points;
    return_points.reserve(width_ * height_);
    int i = 0;
    // std::cout<<"111111111"<<std::endl;
    std::vector<std::vector<Eigen::Vector3f>>::iterator get_row;
    for (get_row = points.begin() + start_rows; get_row != points.begin() + start_rows + height_; get_row++)
    {
      // std::cout<<"i = "<<i<<std::endl;
      std::vector<Eigen::Vector3f>::iterator get_point;
      int j = 0;
      for (get_point = get_row->begin() + start_cols; get_point != get_row->begin() + start_cols + width_; get_point++)
      {
        // std::cout<<"j = "<< j <<std::endl;
        if (!std::isnan(get_point->z()))
        {
          // 行 列
          IndexPoint tmppoint = std::make_pair(std::make_pair(start_rows + i, start_cols + j), *get_point);
          return_points.emplace_back(tmppoint);
        }
        j++;
      }
      i++;
    }
    return return_points;
    // std::vector<Eigen::Vector3f> rectPoints;
    // rectPoints.reserve(width_ * height_);
    // for (size_t i = 0; i < height_; i++)
    // {
    //   std::vector<Eigen::Vector3f> tmp(points.at(i + start_rows).begin() + start_cols, points.at(i + start_rows).begin() + start_cols + width_);
    //   rectPoints.insert(rectPoints.end(), tmp.begin(), tmp.end());
    // }
    // assert(rectPoints.size() == width_ * height_);
    // // std::for_each 可以替换
    // std::vector<Eigen::Vector3f> Points;// 只取有效点
    // for (auto & iter_point : rectPoints)
    // {
    //   if (!std::isnan(iter_point(2)))
    //   {
    //     Points.emplace_back(iter_point);
    //   }
    // }
    // return Points;
  }

  void getRectPointAndIndex(size_t start_rows, size_t start_cols, size_t width, size_t height, std::vector<Eigen::Vector3f> & getpoints, std::vector<std::pair<size_t, size_t>> & getindexs)
  {
    std::vector<std::vector<Eigen::Vector3f>>::iterator get_row;
    int i = 0;
    for (get_row = points.begin() + start_rows; get_row != points.begin() + start_rows + height; get_row++)
    {
      std::vector<Eigen::Vector3f>::iterator get_point;
      int j = 0;
      for (get_point = get_row->begin() + start_cols; get_point != get_row->begin() + start_cols + width; get_point++)
      {
        if (!std::isnan(get_point->z()))
        {
          getpoints.emplace_back(*get_point);
          getindexs.emplace_back(std::make_pair(start_rows + i, start_cols + j));
        }
        j++;
      }
      i++;
    }
  }

  orginazed_points Rect(size_t start_col, size_t start_row, size_t width_, size_t height_)
  {
    std::cout<<"Rect: "<<width_<<" "<<height_<<std::endl;
    orginazed_points return_points;
    return_points.width = width_;
    return_points.height = height_;
    return_points.points.reserve(height_);
    for (size_t i = 0; i < height_; i++)
    {
      std::vector<Eigen::Vector3f> i_row;
      i_row.reserve(width_);
      i_row.insert(i_row.end(), points.at(i + start_row).begin() + start_col, points.at(i + start_row).begin() + start_col + width_);
      assert(i_row.size() == width_);
      return_points.points.emplace_back(i_row);
    }
    return return_points;
  }

  void clear()
  {
    points.clear();
  }

// row_index: index in image
  Eigen::Vector3f getIndexPoint(size_t row_index, size_t col_index)
  {
    return points.at(row_index).at(col_index);
  }

  
};

struct orginazed_points_pcl
{
  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  size_t width_pc, height_pc;
  /**
   * @description: 取矩形区域内的点云，存储到rectPoints
   * @param {size_t} start_xindex 列开始
   * @param {size_t} start_yindex 行开始
   * @param {size_t} width 列宽
   * @param {size_t} height 行宽
   * @param {PointCloud<pcl::PointXYZ>} rectPoints
   * @return {*}
   */  
  void getRectPoints(size_t start_xindex, size_t start_yindex, size_t width, size_t height, pcl::PointCloud<pcl::PointXYZ> &rectPoints)
  {
    rectPoints.width = width; rectPoints.height = height;
    std::cout<<"width_pc: "<<width_pc<<"height_pc: "<<height_pc<<std::endl;
    for (size_t i = 0; i < height; i++)
    {
      size_t start_index = (start_yindex + i) * width_pc + start_xindex;
      size_t end_index = (start_yindex + i) * width_pc + start_xindex + width;
      assert(end_index < pointcloud.size());
      rectPoints.points.insert(rectPoints.points.end(), pointcloud.begin() + start_index, pointcloud.begin() + end_index);
    }
  }
  void clear()
  {
    pointcloud.clear();
  }
};

struct parameter
{
  size_t leafnode_depth;
  size_t leafnode_width;
  float patch_num_percent_th;
  size_t patch_num_th;
  float patch_mse_th;
  float eigen_value_th;
  float merge_normal_dot_th;
  float merge_normal_distance;
  float quatree_merge_normal_dot_th;
  size_t quatree_width;

  void initial(size_t raw_points_width)
  {
    LOG(INFO)<<"raw_points_width: "<<raw_points_width<<std::endl;
    std::cout<<"raw_points_width: "<<raw_points_width<<std::endl;
    patch_num_th = patch_num_percent_th * leafnode_width * leafnode_width;
    // std::cout<<leafnode_width<<std::endl;
    // std::cout<<(double)raw_points_width/(double)leafnode_width<<std::endl;
    // std::cout<<std::log2((double)raw_points_width/(double)leafnode_width)<<std::endl;
    // std::cout<<std::ceil(log2(raw_points_width/leafnode_width))<<std::endl;
    leafnode_depth = std::ceil(log2((double)raw_points_width/(double)leafnode_width));
    std::cout<<"leafnode_depth: "<<leafnode_depth<<endl;
    quatree_width = std::pow(2, leafnode_depth) * leafnode_width;
  }

  void showParameter()
  {
    LOG(INFO)<<"leafnode_depth: "<<leafnode_depth<<endl;
    LOG(INFO)<<"leafnode_width: "<<leafnode_width<<endl;
    LOG(INFO)<<"patch_num_th: "<<patch_num_th<<endl;
    LOG(INFO)<<"patch_mse_th: "<<patch_mse_th<<endl;
    LOG(INFO)<<"eigen_value_th: "<<eigen_value_th<<endl;
    LOG(INFO)<<"merge_normal_dot_th: "<<merge_normal_dot_th<<endl;
    LOG(INFO)<<"merge_normal_distance: "<<merge_normal_distance<<endl;
    LOG(INFO)<<"quatree_merge_normal_dot_th: "<<quatree_merge_normal_dot_th<<endl;
    LOG(INFO)<<"quatree_width: "<<quatree_width<<endl;
  }
};

struct index2D
{
  size_t rows;
  size_t cols;
  std::vector<std::vector<quatree::node*>> index2d;
  index2D()
  {

  }
  index2D(size_t rows_, size_t cols_, quatree::node* p)
  {
    rows = rows_; cols = cols_;
    assert(p->start_rows_2d + p->width_2d >= cols);
    assert(p->start_cols_2d + p->width_2d >= cols);

    initial(p);
  }

  index2D & operator=(const index2D & other)
  {
    if (this == &other)
    {
      return *this;
    }
    this->rows = other.rows;
    this->cols = other.cols;
    this->index2d = other.index2d;
    return *this;
  }

  void initial(quatree::node* root)
  {
    index2d.resize(rows);
    for (auto & iter : index2d)
    {
      iter.resize(cols);
    }
    std::list<quatree::node*> Q;
    Q.emplace_back(root);
    while (!Q.empty())
    {
      quatree::node* tmpnode = Q.front();
      Q.pop_front();
      if (tmpnode->is_plane || (!tmpnode->is_plane && tmpnode->is_leafnode))// 混合区域增长算法需要考虑所有不为nullptr的点
      {
        std::vector<std::vector<quatree::node*>>::iterator rows_iter = index2d.begin() + tmpnode->start_rows_2d;
        for (size_t i = 0; i < tmpnode->width_2d; i++)
        {
          std::vector<quatree::node*>::iterator rows_cols_iter = rows_iter->begin() + tmpnode->start_cols_2d;
          for (size_t j = 0; j < tmpnode->width_2d; j++)
          {
            *rows_cols_iter = tmpnode;
            rows_cols_iter++; 
          }
          rows_iter++;
        }
      }
      else
      {
        for (auto & iter_node : tmpnode->children)
        {
          if (iter_node)
          {
            Q.emplace_back(iter_node);
          }
        }
      }
    }
  }
  // x是行 y是列
  quatree::node* getNodeByIndex(size_t x, size_t y)
  {
    return index2d.at(x).at(y);
  }

  void setNullptr()
  {
    for (auto & iter : index2d)
    {
      std::for_each(iter.begin(), iter.end(), [](quatree::node* & p){ p = nullptr;});
    }
  }

  void showInfo()
  {
    for (auto & iter_rows : index2d)
    {
      for (auto & iter_rows_cols : iter_rows)
      {
        std::cout<<iter_rows_cols<<" ";
      }
      std::cout<<std::endl;
    }
  }

  // 在LOG(INFO)中，即使默认不打印出其内容，也会耗时
  void getNeighbors(quatree::node* p, std::set<quatree::node*, quatree::compnode> & neighbors)
  {
    neighbors.clear();
    // 分三块，上面一排， 中间n排，虽然只有两个数，下面一排
    size_t start_col_index = (p->start_cols_2d > 0) ? p->start_cols_2d - 1 : p->start_cols_2d;
    size_t end_col_index = (p->start_cols_2d + p->width_2d < cols) ? (p->start_cols_2d + p->width_2d) : (p->start_cols_2d + p->width_2d - 1);
    // LOG(INFO)<<"start_col_index: "<<start_col_index<<" end_col_index: "<<end_col_index;
    if (p->start_rows_2d > 0)
    {
      vector<quatree::node*>::iterator iter_node;
      for (iter_node = index2d.at(p->start_rows_2d - 1).begin() + start_col_index; iter_node <= index2d.at(p->start_rows_2d - 1).begin() + end_col_index; iter_node++)
      {
        if (*iter_node)
        {
          neighbors.insert(*iter_node);
        }
      }
    }
    // LOG(INFO)<<"FIRST: ";
    // for (auto & iter_neighbor : neighbors)
    // {
    //   // LOG(INFO)<<iter_neighbor;
    // }
    vector<size_t> col_indexs;
    if (start_col_index == p->start_cols_2d - 1)
    {
      col_indexs.emplace_back(start_col_index);
    }
    if (end_col_index == p->start_cols_2d + p->width_2d)
    {
      col_indexs.emplace_back(end_col_index);
    }
    // LOG(INFO)<<"WIDTH: "<<p->width_2d;
    for (size_t i = 0; i < p->width_2d; i++)
    {      
      for (auto & iter_col_index : col_indexs)
      {
        // LOG(INFO)<<iter_col_index;
        // LOG(INFO)<<*(index2d.at(p->start_rows_2d + i).begin() + iter_col_index);
        quatree::node* tmpnode = *(index2d.at(p->start_rows_2d + i).begin() + iter_col_index);
        if (tmpnode)
        {
          neighbors.insert(tmpnode);
        }
      }
      // LOG(INFO)<<neighbors.size();
    }
    // LOG(INFO)<<"second: ";
    // for (auto & iter_neighbor : neighbors)
    // {
    //   // LOG(INFO)<<iter_neighbor;
    // }
    size_t row_index = p->start_rows_2d + p->width_2d;
    if (row_index < rows)
    {
      vector<quatree::node*>::iterator iter_node;
      for (iter_node = index2d.at(row_index).begin() + start_col_index; iter_node <= index2d.at(row_index).begin() + end_col_index; iter_node++)
      {
        if (*iter_node)
        {
          neighbors.insert(*iter_node);
        }
      }
    }
    // LOG(INFO)<<"tmpnode: "<<p;
    // for (auto & iter_neighbor : neighbors)
    // {
    //   // LOG(INFO)<<iter_neighbor;
    // }
  }
};

struct levelsIndex2d
{
  size_t rows;
  size_t cols;

  typedef std::vector<std::vector<quatree::node*>> levelIndex2d;

  std::vector<levelIndex2d> levelsIndex;

  levelsIndex2d()
  {

  }
  // 必须长宽相等，因为会显示根节点的地址
  levelsIndex2d(quatree::node* p)
  {
    rows = p->start_rows_2d + p->width_2d;
    cols = p->start_cols_2d + p->width_2d;
    initial(p);
  }

  levelsIndex2d & operator=(const levelsIndex2d & other)
  {
    if (this == &other)
    {
      return *this;
    }
    this->rows = other.rows;
    this->cols = other.cols;
    this->levelsIndex = other.levelsIndex;
    return *this;
  }

  void initial(quatree::node* root)
  {
    size_t level = 0;
    std::list<quatree::node*> l;
    l.emplace_back(root);
    levelIndex2d tmpIndex2d;
    tmpIndex2d.resize(rows);
    for (auto & iter_row : tmpIndex2d)
    {
      iter_row.resize(cols);
      std::for_each(iter_row.begin(), iter_row.end(), [](quatree::node* & p){ p = nullptr;});
    }
    
    while (!l.empty())
    {
      quatree::node* tmpnode = l.front();
      
      if (tmpnode->depth == level)
      {
        std::vector<std::vector<quatree::node*>>::iterator rows_iter = tmpIndex2d.begin() + tmpnode->start_rows_2d;
        for (size_t i = 0; i < tmpnode->width_2d; i++)
        {
          std::vector<quatree::node*>::iterator rows_cols_iter = rows_iter->begin() + tmpnode->start_cols_2d;
          for (size_t j = 0; j < tmpnode->width_2d; j++)
          {
            *rows_cols_iter = tmpnode;
            rows_cols_iter++;
          }
          rows_iter++;
        }
        for (auto & iter_node : tmpnode->children)
        {
          if (iter_node)
          {
            l.emplace_back(iter_node);
          }
        }
        l.pop_front();
      }
      else
      {
        levelsIndex.emplace_back(tmpIndex2d);
        for (auto & iter_row : tmpIndex2d)
        {
          std::for_each(iter_row.begin(), iter_row.end(), [](quatree::node* & p){ p = nullptr;});
        }
        level++;
      }
    }
  }

  void showInfo()
  {
    for (auto & iter_index2d : levelsIndex)
    {
      for (auto & iter_rows : iter_index2d)
      {
        for (auto & iter_rows_cols : iter_rows)
        {
          std::cout<<iter_rows_cols<<" ";
        }
        std::cout<<std::endl;
      }
      std::cout<<std::endl;
    }
  }
};


struct index2dBinaryCode
{
  size_t row_cor, col_cor;
  size_t rows, cols;
  std::vector<std::vector<quatree::node*>> index2ds;
  index2dBinaryCode(size_t rows_, size_t cols_, quatree::node* p)
  {
    rows = rows_; cols = cols;
    size_t depth = std::log2(p->width_2d);
    row_cor = 0; col_cor = 0;
    for (size_t i = 0; i < depth; i++)
    {
      row_cor = (row_cor<<2)|(1<<2);
      col_cor = (col_cor<<2)|(1);
    }
    index2ds.resize(rows);
    for (auto & iter_row : index2ds)
    {
      iter_row.resize(cols);
    }
    initial(p);
  }
  void initial(quatree::node* p)
  {
    // std::list<quatree::node*> l;
    // l.emplace_back(p);
    // while (!l.empty())
    // {
    //   quatree::node* tmpnode = l.front();
    //   l.pop_front();
    //   if (tmpnode->is_leafnode)
    //   {
    //     size_t row_index = tmpnode->binary_code & row_cor;
    //     size_t col_index = tmpnode->binary_code & col_cor;
    //     index2ds.at(row_index).at(col_index) = tmpnode;
    //   }
    //   else
    //   {
    //     for (auto & iter_child : tmpnode->children)
    //     {
    //       if (iter_child)
    //       {
    //         l.emplace_back(iter_child);
    //       }
    //     }
    //   }
    // }
  }
};

struct plane_info
{
  Eigen::Vector3f center;
  Eigen::Vector3f normal;
  std::vector<std::vector<Eigen::Vector3f>> contours;
  std::vector< std::array<int, 4> > hierarchy;
  plane_info transform(Eigen::Matrix4f & T)
  {
    plane_info tmpplane;
    tmpplane.hierarchy = hierarchy;
    Eigen::Vector4f center_H;
    center_H.head(3) = center; center_H(3) = 1;
    tmpplane.center = (T * center_H).head(3);

    tmpplane.normal = T.block<3,3>(0,0) * normal;

    for (auto & contour : contours)
    {
      vector<Eigen::Vector3f> contour_T;
      contour_T.reserve(contour.size());
      for (auto & point : contour)
      {
        // std::cout<<"befor transform: "<<point.transpose()<<endl;
        Eigen::Vector4f point_H;
        point_H.head(3) = point; point_H(3) = 1;
        // std::cout<<"after transfrom: "<<(T*point_H).head(3).transpose()<<endl;
        contour_T.emplace_back((T * point_H).head(3));
      }
      tmpplane.contours.emplace_back(contour_T);
    }
    return tmpplane;
  }
};

Eigen::Matrix4f getT(const float &px, const float &py, const float &pz, const float &rx, const float &ry, const float &rz);


double _deg2rad(double degree);

float correct2threeDecimal(float x);

#endif