#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "lwa4p_calibration_poses/joint_pose_file.hpp"

using lwa4p_calibration_poses::JointPoseFile;

namespace
{

constexpr std::size_t JOINT_COUNT = 3;

class JointPoseFileTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto * test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    path_ = std::filesystem::temp_directory_path() /
      ("joint_pose_file_" + std::string(test_info->name()) + ".txt");
    std::filesystem::remove(path_);
  }

  void TearDown() override
  {
    std::filesystem::remove(path_);
  }

  void write_file(const std::string & content) const
  {
    std::ofstream file(path_);
    file << content;
  }

  std::filesystem::path path_;
};

}  // namespace

TEST_F(JointPoseFileTest, AppendedPosesAreReadBackInOrder)
{
  const JointPoseFile pose_file(path_.string(), JOINT_COUNT);
  pose_file.append({0.1, 0.2, 0.3});
  pose_file.append({-1.0, 0.0, 1.5});

  const std::vector<std::vector<double>> expected = {{0.1, 0.2, 0.3}, {-1.0, 0.0, 1.5}};
  EXPECT_EQ(pose_file.read_all(), expected);
}

TEST_F(JointPoseFileTest, IgnoresCommentsAndBlankLines)
{
  write_file("# taught on the table\n\n0.1 0.2 0.3  # board left\n   \n0.4 0.5 0.6\n");
  const JointPoseFile pose_file(path_.string(), JOINT_COUNT);

  const std::vector<std::vector<double>> expected = {{0.1, 0.2, 0.3}, {0.4, 0.5, 0.6}};
  EXPECT_EQ(pose_file.read_all(), expected);
}

TEST_F(JointPoseFileTest, RejectsLineWithWrongJointCount)
{
  write_file("0.1 0.2 0.3\n0.4 0.5\n");
  const JointPoseFile pose_file(path_.string(), JOINT_COUNT);

  EXPECT_THROW(pose_file.read_all(), std::runtime_error);
}

TEST_F(JointPoseFileTest, RejectsLineWithText)
{
  write_file("0.1 abc 0.3\n");
  const JointPoseFile pose_file(path_.string(), JOINT_COUNT);

  EXPECT_THROW(pose_file.read_all(), std::runtime_error);
}

TEST_F(JointPoseFileTest, RejectsMissingFile)
{
  const JointPoseFile pose_file(path_.string(), JOINT_COUNT);

  EXPECT_THROW(pose_file.read_all(), std::runtime_error);
}

TEST_F(JointPoseFileTest, RejectsAppendWithWrongJointCount)
{
  const JointPoseFile pose_file(path_.string(), JOINT_COUNT);

  EXPECT_THROW(pose_file.append({0.1, 0.2}), std::invalid_argument);
}

TEST(JointPoseFilePathTest, ExpandsHomeDirectory)
{
  const JointPoseFile pose_file("~/poses.txt", JOINT_COUNT);

  EXPECT_EQ(pose_file.path(), std::string(std::getenv("HOME")) + "/poses.txt");
}

TEST(JointPoseFilePathTest, RejectsZeroJointCount)
{
  EXPECT_THROW(JointPoseFile("poses.txt", 0), std::invalid_argument);
}
