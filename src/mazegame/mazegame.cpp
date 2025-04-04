#include "mazegame.h"

#define ROBOT_RADIUS 2

std::vector<MazeGame::line_t> MazeGame::penalty_list, MazeGame::smup_list;
int MazeGame::num_smups = 0;
int MazeGame::num_penalties = 0;

void MazeGame::init_boundary_lines()
{
  // DQ lines
  smup_list.push_back({{23.5, 0}, {23.5, 47}});
  smup_list.push_back({{23.5, 47}, {47, 47}});
  smup_list.push_back({{47, 47}, {47, 118}});

  // Penalty Lines
  penalty_list.push_back({{0, 68}, {21, 68}});
  penalty_list.push_back({{23, 92}, {44, 92}});
  penalty_list.push_back({{23, 92}, {23, 114}});
  penalty_list.push_back({{23, 114}, {70, 114}});
  penalty_list.push_back({{50, 22}, {75, 22}});
  penalty_list.push_back({{75, 23}, {75, 40}});
}

bool MazeGame::is_single_penalty()
{
  static bool was_prev_penalty = false;

  Pose2d pos = odom.get_position();
  std::vector<Translation2d> intersect;

  bool is_penalty = false;
  for(line_t l : penalty_list)
  {
    intersect = PurePursuit::line_circle_intersections({pos.x(), pos.y()}, ROBOT_RADIUS, l.point1, l.point2);

    if(intersect.size() > 0)
    {
      is_penalty = true;
      break;
    }
  }

  if(!was_prev_penalty && is_penalty)
  {
    // main_controller.rumble(".");
    num_penalties++;
  }
  
  was_prev_penalty = is_penalty;

  return is_penalty;
}

bool MazeGame::is_super_mega_ultra_penalty()
{
  static bool was_prev_penalty = false;

  Pose2d pos = odom.get_position();
  std::vector<Translation2d> intersect;

  bool is_penalty = false;
  for(line_t l : smup_list)
  {
    intersect = PurePursuit::line_circle_intersections({pos.x(), pos.y()}, ROBOT_RADIUS, l.point1, l.point2);

    if(intersect.size() > 0)
    {
      is_penalty = true;
      break;
    }
  }

  if(!was_prev_penalty && is_penalty)
    num_smups++;
  
  was_prev_penalty = is_penalty;

  return is_penalty;
}