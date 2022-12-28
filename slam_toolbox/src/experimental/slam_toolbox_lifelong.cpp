/*
 * slam_toolbox
 * Copyright (c) 2019, Samsung Research America
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#include "slam_toolbox/experimental/slam_toolbox_lifelong.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
void LifelongSlamToolbox::checkIsNotNormalized(const double& value)
/*****************************************************************************/
{
  if(value < 0.0 || value > 1.0)
  {
    ROS_FATAL("All stores and scales must be in range [0, 1].");
    exit(-1);
  }
}

/*****************************************************************************/
LifelongSlamToolbox::LifelongSlamToolbox(ros::NodeHandle& nh)
: SlamToolbox(nh),
  last_scan_(NULL),
  lifelong_scan_matcher_(NULL)
/*****************************************************************************/
{
  loadPoseGraphByParams(nh);
  nh.param("lifelong_debug_mode", debug_mode_, true);
  nh.param("lifelong_partial_remove", partial_remove_, true);
  nh.param("lifelong_search_maximum_distance", search_max_distance_,20.0);
  nh.param("lifelong_scan_match_maximum_range",scan_match_max_range_, 10.0);
  nh.param("lifelong_node_buffer_size",node_buffer_size_, 10);
  nh.param("lifelong_node_maximum_distance",node_maximum_distance_, 5.0);
  nh.param("lifelong_match_radius", match_radius_, 0.1);
  nh.param("lifelong_search_use_tree", use_tree_, false);
  nh.param("lifelong_minimum_score", iou_thresh_, 0.10);
  nh.param("lifelong_iou_match", iou_match_, 0.85);
  nh.param("lifelong_node_removal_score", removal_score_, 0.10);
  nh.param("lifelong_overlap_score_scale", overlap_scale_, 0.5);
  nh.param("lifelong_constraint_multiplier", constraint_scale_, 0.05);
  nh.param("lifelong_nearby_penalty", nearby_penalty_, 0.001);
  nh.param("lifelong_candidates_scale", candidates_scale_, 0.03);

  checkIsNotNormalized(iou_thresh_);
  checkIsNotNormalized(constraint_scale_);
  checkIsNotNormalized(removal_score_);
  checkIsNotNormalized(overlap_scale_);
  checkIsNotNormalized(iou_match_);
  checkIsNotNormalized(nearby_penalty_);
  checkIsNotNormalized(candidates_scale_);

  ROS_WARN("Lifelong mapping mode in SLAM Toolbox is considered "
    "experimental and should be understood before proceeding. Please visit: "
    "https://github.com/SteveMacenski/slam_toolbox/wiki/"
    "Experimental-Lifelong-Mapping-Node for more information.");

  // in lifelong mode, we cannot have interactive mode enabled
  enable_interactive_mode_ = false;

  // add publisher to check removed vertices
  removed_node_pub_ = nh.advertise<visualization_msgs::MarkerArray>("karto_removed_graph_visualization",1);
  threads_.push_back(std::make_unique<boost::thread>(
    boost::bind(&LifelongSlamToolbox::publishRemovedGraph, this)));

  ssDebug_ = nh.advertiseService("lifelong_debug_mode", &LifelongSlamToolbox::debugModeCallback, this);
}

/*****************************************************************************/
void LifelongSlamToolbox::laserCallback(
  const sensor_msgs::LaserScan::ConstPtr& scan)
/*****************************************************************************/
{
  // no odom info
  Pose2 pose;
  if(!pose_helper_->getOdomPose(pose, scan->header.stamp))
  {
    return;
  }

  // ensure the laser can be used
  LaserRangeFinder* laser = getLaser(scan);

  if(!laser)
  {
    ROS_WARN_THROTTLE(5., "Failed to create laser device for"
      " %s; discarding scan", scan->header.frame_id.c_str());
    return;
  }

  // LTS additional bounded node increase parameter (rate, or total for run or at all?)
  // LTS pseudo-localization mode. If want to add a scan, but not deleting a scan, add to local buffer?
  // LTS if(eval() && dont_add_more_scans) {addScan()} else {localization_add_scan()}
  // LTS if(eval() && ctr / total < add_rate_scans) {addScan()} else {localization_add_scan()}
  karto::LocalizedRangeScan* range_scan = addScan(laser, scan, pose);

  // process to remove old vertex or scan points
  if(range_scan)
  {
    mapper_ = smapper_->getMapper();

    evaluateNodeDepreciation(range_scan);

    if(partial_remove_)
    {
      partialGraphModification(range_scan, last_scan_);
    }

    last_scan_ = mapper_->GetMapperSensorManager()->GetLastScan(range_scan->GetSensorName());     
  }

  return;
}

/*****************************************************************************/
void LifelongSlamToolbox::partialGraphModification(
  LocalizedRangeScan* range_scan,
  LocalizedRangeScan* last_scan)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(smapper_mutex_);

  if(last_scan == NULL)
  {
    return;
  }

  if(!mapper_->HasMovedEnough(range_scan, last_scan))
  {
    return;
  }

  LocalizedRangeScanVector near_linked_scans, near_linked_scans_tmp;
  LocalizedRangeScanVector::iterator near_scan_it;

  // Fine near scans
  // near_linked_scans = mapper_->GetGraph()->FindNearLinkedScans(range_scan, search_max_distance_);
  near_linked_scans = mapper_->GetGraph()->FindNearByScans( range_scan->GetSensorName(), 
                                                            range_scan->GetSensorPose(), search_max_distance_);
  near_linked_scans_tmp = near_linked_scans;

  // Check if it is a previously visited vertex or out of bound
  for (near_scan_it = near_linked_scans_tmp.begin(); near_scan_it != near_linked_scans_tmp.end();)  
  {
    LocalizedRangeScan* near_scan = *near_scan_it;
    if( range_scan->GetUniqueId() - near_scan->GetUniqueId() < node_buffer_size_ ||
        (range_scan->GetSensorPose().GetPosition() - near_scan->GetSensorPose().GetPosition()).Length() > node_maximum_distance_)
    {
      near_scan_it = near_linked_scans_tmp.erase(near_scan_it);
    }
    else
    {
      near_scan_it++;
    }
  }

  if(!near_linked_scans_tmp.size())
  {
    return;
  }

  // Get removed grid point
  PointVectorDoubleWithIndex removed_grid_point = findRemovedGridPoint(range_scan, near_linked_scans);

  // Update scans & vertices
  for (near_scan_it = near_linked_scans.begin(); near_scan_it != near_linked_scans.end(); ++near_scan_it)
  {
    LocalizedRangeScan* near_scan = *near_scan_it;

    Vertex<LocalizedRangeScan>* near_vertex = mapper_->GetGraph()->GetVertex(near_scan);

    if(near_vertex == nullptr || near_scan == nullptr || range_scan == *near_scan_it)
    {
      continue;
    }

    if (near_scan->GetPointReadings(true).size() == 0)
    {
      removeFromSlamGraph(near_vertex); // remove invalid vertices
    }
    else
    {
      removeInvalidReadings(near_vertex, removed_grid_point);   
      updateScansFromSlamGraph(near_vertex);
    }
  }

  return;
}

/*****************************************************************************/
PointVectorDoubleWithIndex LifelongSlamToolbox::findRemovedGridPoint(
  LocalizedRangeScan* range_scan,
  std::vector<karto::LocalizedRangeScan*> near_linked_scan)
/*****************************************************************************/
{ 
  if(!lifelong_scan_matcher_)
  {
    lifelong_scan_matcher_ = ScanMatcher::Create(mapper_, mapper_->getParamCorrelationSearchSpaceDimension(),
                                                  mapper_->getParamCorrelationSearchSpaceResolution(),
                                                  mapper_->getParamCorrelationSearchSpaceSmearDeviation(), 
                                                  scan_match_max_range_);
  }

  // Set offset
  CorrelationGrid* correlation_grid = lifelong_scan_matcher_->GetCorrelationGrid();
  Rectangle2<kt_int32s> roi = correlation_grid->GetROI();

  Vector2<kt_double> offset;
  Pose2 scan_pose = range_scan->GetSensorPose();
  offset.SetX(scan_pose.GetX() - (0.5 * (roi.GetWidth() - 1) * correlation_grid->GetResolution()));
  offset.SetY(scan_pose.GetY() - (0.5 * (roi.GetHeight() - 1) * correlation_grid->GetResolution()));

  correlation_grid->GetCoordinateConverter()->SetOffset(offset); 

  // Get occupied grid points
  PointVectorDoubleWithIndex occupied_grid_point;
  PointVectorDoubleWithIndex removed_grid_point;

  for (int scan_idx = 0; scan_idx<near_linked_scan.size(); scan_idx++)
  {
    PointVectorDoubleWithIndex valid_points = lifelong_scan_matcher_->FindValidPoints(near_linked_scan[scan_idx], 
                                                                                      scan_pose.GetPosition());
    for (int point_idx = 0; point_idx<valid_points.size(); point_idx++)
    {
      Vector2<kt_int32s> grid_point = correlation_grid->WorldToGrid(valid_points[point_idx].second);
      if (!math::IsUpTo(grid_point.GetX(), roi.GetWidth()) ||
          !math::IsUpTo(grid_point.GetY(), roi.GetHeight()))
      {
        continue;
      }

      int gird_index = correlation_grid->GridIndex(grid_point);

      if (correlation_grid->GetDataPointer()[gird_index] == GridStates_Occupied)
      {
        continue;
      }

      occupied_grid_point.push_back(std::pair<int,Vector2<kt_double>>(valid_points[point_idx].first, valid_points[point_idx].second));
      correlation_grid->GetDataPointer()[gird_index] = GridStates_Occupied;

      correlation_grid->SmearPoint(grid_point);
    }
  }

  // Find removed grid point
  for (int occupied_idx = 0; occupied_idx<occupied_grid_point.size(); occupied_idx++)
  {
    Vector2<kt_double> point_value = occupied_grid_point[occupied_idx].second;

    Pose2 scan_pose = range_scan->GetSensorPose();

    kt_double scan_start_angle = scan_pose.GetHeading() + range_scan->GetLaserRangeFinder()->GetMinimumAngle();
    kt_double scan_end_angle = scan_pose.GetHeading() + range_scan->GetLaserRangeFinder()->GetMaximumAngle();
    kt_double point_angle = atan2(point_value.GetY()-scan_pose.GetY(), point_value.GetX()-scan_pose.GetX());

    // Excluding points outside the scan measurement range
    if(point_angle < scan_start_angle || point_angle > scan_end_angle)
    {
      continue;
    }

    // Excluding points that are farther than current scan point
    kt_double angle_diff = math::NormalizeAngle(point_angle - scan_start_angle);

    if(angle_diff < 0)
    {
      angle_diff += 2*M_PI;
    }

    int point_index = (int)(angle_diff / (range_scan->GetLaserRangeFinder()->GetAngularResolution()));
    kt_double point_dist = (range_scan->GetSensorPose().GetPosition() - point_value).Length();

    int is_filtered = false;
    for(int check_index = -2; check_index <= 2; check_index++)
    {
      if(range_scan->GetRangeReadings()[point_index + check_index] <= point_dist)
      {
        is_filtered = true;
        continue;
      }
    }
    if(is_filtered)
    {
      continue;
    }

    // Check that point exists
    PointVectorDoubleWithIndex point_readings = range_scan->GetPointReadings();

    auto iter = std::find_if(point_readings.begin(), point_readings.end(), CheckIsExist(point_value, match_radius_));
    if(iter == point_readings.end()) // if not exist
    {
      removed_grid_point.push_back(occupied_grid_point[occupied_idx]);
    }
  }

  return removed_grid_point;
}

/*****************************************************************************/
void LifelongSlamToolbox::removeInvalidReadings(
  Vertex<LocalizedRangeScan>* vertex,
  PointVectorDoubleWithIndex removed_grid_point)
/*****************************************************************************/
{
  // Set removed point to -1 [m]
  PointVectorDoubleWithIndex vertex_point_readings = vertex->GetObject()->GetPointReadings();

  kt_double* range_readings = vertex->GetObject()->GetRangeReadings();
  int num_range_readings = vertex->GetObject()->GetNumberOfRangeReadings();

  for (int removed_idx = 0; removed_idx < removed_grid_point.size(); removed_idx++)
  {

    Vector2<kt_double> removed_point = removed_grid_point[removed_idx].second;
    auto reading_iter = std::find_if(vertex_point_readings.begin(), 
                                    vertex_point_readings.end(), 
                                    CheckIsExist(removed_point, match_radius_));

    if(reading_iter != vertex_point_readings.end())
    {
      range_readings[reading_iter->first] = -1;

      // Searching near scan points
      for(int check_index = 1; check_index < 10; check_index ++)
      {
        Vector2<kt_double> delta = (reading_iter + check_index)->second  - removed_point;
        if(delta.Length() <= match_radius_)
        {
          range_readings[(reading_iter + check_index)->first] = -1;
        }
      }
    }
  }

  // Update readings
  vertex->GetObject()->SetRangeReadings(RangeReadingsVector(range_readings,
                                                            range_readings + num_range_readings));
  vertex->GetObject()->UpdateReadings();

  return;
}

/*****************************************************************************/
void LifelongSlamToolbox::updateScansFromSlamGraph(
  Vertex<LocalizedRangeScan>* vertex)
/*****************************************************************************/
{
  mapper_->GetMapperSensorManager()->EditScan(vertex->GetObject());
  mapper_->GetMapperSensorManager()->EditRunningScans(vertex->GetObject());
  dataset_->EditData(vertex->GetObject());
  mapper_->GetGraph()->EditVertex(vertex->GetObject()->GetSensorName(), vertex);

  return;
}

/*****************************************************************************/
void LifelongSlamToolbox::evaluateNodeDepreciation(
  LocalizedRangeScan* range_scan)
/*****************************************************************************/
{
  boost::mutex::scoped_lock lock(smapper_mutex_);

  const BoundingBox2& bb = range_scan->GetBoundingBox();
  const Size2<double> bb_size = bb.GetSize();
  double radius = sqrt(bb_size.GetWidth()*bb_size.GetWidth() +
    bb_size.GetHeight()*bb_size.GetHeight()) / 2.0;
  Vertices near_scan_vertices = FindScansWithinRadius(range_scan, radius);

  ScoredVertices scored_verices =
    computeScores(near_scan_vertices, range_scan);

  ScoredVertices::iterator it;
  for (it = scored_verices.begin(); it != scored_verices.end(); ++it)
  {
    if(it->GetScore() < removal_score_)
    {
      ROS_INFO("Removing node %i from graph with score: %f and "
        "old score: %f.", it->GetVertex()->GetObject()->GetUniqueId(),
        it->GetScore(), it->GetVertex()->GetScore());
      removeFromSlamGraph(it->GetVertex());

    }
    else
    {
      updateScoresSlamGraph(it->GetScore(), it->GetVertex());
    }
  }

  return;
}

/*****************************************************************************/
Vertices LifelongSlamToolbox::FindScansWithinRadius(
  LocalizedRangeScan* scan, const double& radius)
/*****************************************************************************/
{
  // Using the tree will create a Kd-tree and find all neighbors in graph
  // around the reference scan. Otherwise it will use the graph and
  // access scans within radius that are connected with constraints to this
  // node.

  if(use_tree_)
  {
    return
      mapper_->GetGraph()->FindNearByVertices(
      scan->GetSensorName(), scan->GetBarycenterPose(), radius);
  }
  else
  {
    return 
      mapper_->GetGraph()->FindNearLinkedVertices(scan, radius);
  }
}

/*****************************************************************************/
double LifelongSlamToolbox::computeObjectiveScore(
  const double& intersect_over_union,
  const double& area_overlap,
  const double& reading_overlap,
  const int& num_constraints,
  const double& initial_score,
  const int& num_candidates) const
/*****************************************************************************/
{
  // We have some useful metrics. lets make a new score
  // intersect_over_union: The higher this score, the better aligned in scope these scans are
  // area_overlap: The higher, the more area they share normalized by candidate size
  // reading_overlap: The higher, the more readings of the new scan the candidate contains
  // num_constraints: The lower, the less other nodes may rely on this candidate
  // initial_score: Last score of this vertex before update

  // this is a really good fit and not from a loop closure, lets just decay
  if(intersect_over_union > iou_match_ && num_constraints < 3)
  {
    return -1.0;
  }

  // to be conservative, lets say the overlap is the lesser of the
  // area and proportion of laser returns in the intersecting region.
  double overlap = overlap_scale_ * std::min(area_overlap, reading_overlap);

  // if the num_constraints are high we want to stave off the decay, but not override it
  double contraint_scale_factor = std::min(1.0,
    std::max(0., constraint_scale_ * (num_constraints - 2)));
  contraint_scale_factor = std::min(contraint_scale_factor, overlap);

  //
  double candidates = num_candidates - 1;
  double candidate_scale_factor = candidates_scale_ * candidates;

  // Give the initial score a boost proportional to the number of constraints
  // Subtract the overlap amount, apply a penalty for just being nearby
  // and scale the entire additional score by the number of candidates
  double score =
    initial_score * (1.0 + contraint_scale_factor)
    - overlap
    - nearby_penalty_;

  //score += (initial_score - score) * candidate_scale_factor; 
  
  if(score > 1.0)
  {
    ROS_ERROR("Objective function calculated for vertex score (%0.4f)"
      " greater than one! Thresholding to 1.0", score);
    return 1.0;
  }

  return score;
}

/*****************************************************************************/
double LifelongSlamToolbox::computeScore(
  LocalizedRangeScan* reference_scan,
  Vertex<LocalizedRangeScan>* candidate,
  const double& initial_score, const int& num_candidates)
/*****************************************************************************/
{
  double new_score = initial_score;
  LocalizedRangeScan* candidate_scan = candidate->GetObject();

  // compute metrics for information loss normalized
  double iou = computeIntersectOverUnion(reference_scan, candidate_scan);
  double area_overlap = computeAreaOverlapRatio(reference_scan, candidate_scan);
  int num_constraints = candidate->GetEdges().size();
  double reading_overlap = computeReadingOverlapRatio(reference_scan, candidate_scan);

  bool critical_lynchpoint = candidate_scan->GetUniqueId() == 0 ||
    candidate_scan->GetUniqueId() == 1;
  int id_diff = reference_scan->GetUniqueId() - candidate_scan->GetUniqueId();
  if(id_diff < mapper_->getParamScanBufferSize() ||
    critical_lynchpoint)
  {
    return initial_score;
  }

  double score = computeObjectiveScore(iou,
                               area_overlap,
                               reading_overlap,
                               num_constraints,
                               initial_score,
                               num_candidates);

  ROS_INFO("Metric Scores: Initial: %f, IOU: %f,"
    " Area: %f, Num Con: %i, Reading: %f, outcome score: %f.",
    initial_score, iou, area_overlap, num_constraints, reading_overlap, score);
  return score;
}

/*****************************************************************************/
ScoredVertices
LifelongSlamToolbox::computeScores(
  Vertices& near_scans,
  LocalizedRangeScan* range_scan)
/*****************************************************************************/
{
  ScoredVertices scored_vertices;
  scored_vertices.reserve(near_scans.size());

  // must have some minimum metric to utilize
  // IOU will drop sharply with fitment, I'd advise not setting this value
  // any higher than 0.15. Also check this is a linked constraint
  // We want to do this early to get a better estimate of local candidates
  ScanVector::iterator candidate_scan_it;
  double iou = 0.0;
  for (candidate_scan_it = near_scans.begin();
    candidate_scan_it != near_scans.end(); )
  {
    iou = computeIntersectOverUnion(range_scan,
      (*candidate_scan_it)->GetObject());
    if(iou < iou_thresh_ || (*candidate_scan_it)->GetEdges().size() < 2)
    {
      candidate_scan_it = near_scans.erase(candidate_scan_it);
    }
    else
    {
      ++candidate_scan_it;
    }
  }

  for (candidate_scan_it = near_scans.begin();
    candidate_scan_it != near_scans.end(); ++candidate_scan_it)
  {
    ScoredVertex scored_vertex((*candidate_scan_it),
      computeScore(range_scan, (*candidate_scan_it),
        (*candidate_scan_it)->GetScore(), near_scans.size()));
    scored_vertices.push_back(scored_vertex);
  }
  return scored_vertices;
}

/*****************************************************************************/
void LifelongSlamToolbox::removeFromSlamGraph(
  Vertex<LocalizedRangeScan>* vertex)
/*****************************************************************************/
{
   // publish removed vertex
  if(debug_mode_)
  {
    std::unordered_map<int, Eigen::Vector3d>* graph = mapper_->getScanSolver()->getGraph();
    GraphIterator graphit = graph->find(vertex->GetObject()->GetUniqueId());
    visualization_msgs::Marker m = vis_utils::toMarker(map_frame_, "slam_toolbox", 0.1, {0.0, 1.0, 0.0});
    m.id = graphit->second(0);
    m.pose.position.x = graphit->second(0);
    m.pose.position.y = graphit->second(1);
    removed_array_.markers.push_back(m);   
  }

  mapper_->RemoveNodeFromGraph(vertex);
  mapper_->GetMapperSensorManager()->RemoveScan(
    vertex->GetObject());
  dataset_->RemoveData(vertex->GetObject());
  vertex->RemoveObject();
  delete vertex;
  vertex = nullptr;
  // LTS what do we do about the contraints that node had about it?Nothing?Transfer?
}

/*****************************************************************************/
void LifelongSlamToolbox::updateScoresSlamGraph(const double& score, 
  Vertex<LocalizedRangeScan>* vertex)
/*****************************************************************************/
{
  // Saved in graph so it persists between sessions and runs
  vertex->SetScore(score);
}

/*****************************************************************************/
bool LifelongSlamToolbox::deserializePoseGraphCallback(
  slam_toolbox_msgs::DeserializePoseGraph::Request& req,
  slam_toolbox_msgs::DeserializePoseGraph::Response& resp)
/*****************************************************************************/
{
  if(req.match_type == procType::LOCALIZE_AT_POSE)
  {
    ROS_ERROR("Requested a localization deserialization "
      "in non-localization mode.");
    return false;
  }

  return SlamToolbox::deserializePoseGraphCallback(req, resp);
}

/*****************************************************************************/
void LifelongSlamToolbox::computeIntersectBounds(
  LocalizedRangeScan* s1, LocalizedRangeScan* s2,
  double& x_l, double& x_u, double& y_l, double& y_u)
/*****************************************************************************/
{
  Size2<double> bb1 = s1->GetBoundingBox().GetSize();
  Size2<double> bb2 = s2->GetBoundingBox().GetSize();
  Pose2 pose1 = s1->GetBarycenterPose();
  Pose2 pose2 = s2->GetBarycenterPose();

  const double s1_upper_x = pose1.GetX() + (bb1.GetWidth()  / 2.0);
  const double s1_upper_y = pose1.GetY() + (bb1.GetHeight() / 2.0);
  const double s1_lower_x = pose1.GetX() - (bb1.GetWidth()  / 2.0);
  const double s1_lower_y = pose1.GetY() - (bb1.GetHeight() / 2.0);

  const double s2_upper_x = pose2.GetX() + (bb2.GetWidth()  / 2.0);
  const double s2_upper_y = pose2.GetY() + (bb2.GetHeight() / 2.0);
  const double s2_lower_x = pose2.GetX() - (bb2.GetWidth()  / 2.0);
  const double s2_lower_y = pose2.GetY() - (bb2.GetHeight() / 2.0);

  x_u = std::min(s1_upper_x, s2_upper_x);
  y_u = std::min(s1_upper_y, s2_upper_y);
  x_l = std::max(s1_lower_x, s2_lower_x);
  y_l = std::max(s1_lower_y, s2_lower_y);
  return;
}

/*****************************************************************************/
double LifelongSlamToolbox::computeIntersect(LocalizedRangeScan* s1, 
  LocalizedRangeScan* s2)
/*****************************************************************************/
{
  double x_l, x_u, y_l, y_u;
  computeIntersectBounds(s1, s2, x_l, x_u, y_l, y_u);
  const double intersect = (y_u - y_l) * (x_u - x_l);

  if(intersect < 0.0)
  {
    return 0.0;
  }

  return intersect;
}

/*****************************************************************************/
double LifelongSlamToolbox::computeIntersectOverUnion(LocalizedRangeScan* s1, 
  LocalizedRangeScan* s2)
/*****************************************************************************/
{
  // this is a common metric in machine learning used to determine
  // the fitment of a set of bounding boxes. Its response sharply
  // drops by box matches.

  const double intersect = computeIntersect(s1, s2);

  Size2<double> bb1 = s1->GetBoundingBox().GetSize();
  Size2<double> bb2 = s2->GetBoundingBox().GetSize();
  const double uni = (bb1.GetWidth() * bb1.GetHeight()) +
    (bb2.GetWidth() * bb2.GetHeight()) - intersect;

  return intersect / uni;
}

/*****************************************************************************/
double LifelongSlamToolbox::computeAreaOverlapRatio(
  LocalizedRangeScan* ref_scan, 
  LocalizedRangeScan* candidate_scan)
/*****************************************************************************/
{
  // ref scan is new scan, candidate scan is potential for decay
  // so we want to find the ratio of space of the candidate scan 
  // the reference scan takes up

  double overlap_area = computeIntersect(ref_scan, candidate_scan);
  Size2<double> bb_candidate = candidate_scan->GetBoundingBox().GetSize();
  const double candidate_area = 
    bb_candidate.GetHeight() * bb_candidate.GetWidth();

  return overlap_area / candidate_area;
}

/*****************************************************************************/
double LifelongSlamToolbox::computeReadingOverlapRatio(
  LocalizedRangeScan* ref_scan, 
  LocalizedRangeScan* candidate_scan)
/*****************************************************************************/
{
  const PointVectorDoubleWithIndex& pts = candidate_scan->GetPointReadings(true);
  const int num_pts = pts.size();

  // get the bounds of the intersect area
  double x_l, x_u, y_l, y_u;
  computeIntersectBounds(ref_scan, candidate_scan, x_l, x_u, y_l, y_u);

  PointVectorDoubleWithIndex::const_iterator pt_it;
  int inner_pts = 0;
  for (pt_it = pts.begin(); pt_it != pts.end(); ++pt_it)
  {
    if(pt_it->second.GetX() < x_u && pt_it->second.GetX() > x_l &&
        pt_it->second.GetY() < y_u && pt_it->second.GetY() > y_l)
    {
      inner_pts++;
    }
  }

  return double(inner_pts) / double(num_pts);
}

/*****************************************************************************/
void LifelongSlamToolbox::publishRemovedGraph()
/*****************************************************************************/
{
    double map_update_interval;
    if(!nh_.getParam("map_update_interval", map_update_interval))
    {
      map_update_interval = 10.0;
    }

    ros::Rate r(1.0/map_update_interval);
    while(ros::ok())
    {
      if(debug_mode_)
      {
        removed_node_pub_.publish(removed_array_);
      }

      r.sleep();
    }

  return;
}

/*****************************************************************************/
bool LifelongSlamToolbox::debugModeCallback(
  std_srvs::SetBool::Request& req,
  std_srvs::SetBool::Response& res)
/*****************************************************************************/
{
  if(req.data)
  {
    debug_mode_ = true;
    res.success = true;
    res.message = "Start to publish topic 'karto_removed_graph_visualization'";
  }
  else
  {
    debug_mode_ = false;
    res.success = true;
    res.message = "Finish to publish topic 'karto_removed_graph_visualization'";
  }

  return true;
}

} // end namespace
