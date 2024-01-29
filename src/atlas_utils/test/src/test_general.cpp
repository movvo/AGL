/*
   Copyright 2023 @ MOVVO ROBOTICS
   ---------------------------------------------------------
   Authors: Albert Arlà, Martí Bolet, Bernat Gaston, María Mercadé
   Contact: support.idi@movvo.eu
*/
#pragma GCC diagnostic ignored "-Wpedantic"
#include "test_general.hpp"

using ::testing::Return;

TEST_F(TestGeneral, GetFullPath_CorrectPathWithPrefix)
{
  std::string result = atlas_utils::utils::GetFullPath("package://atlas_utils/test/data/data.txt");
  std::string expected_path = ament_index_cpp::get_package_share_directory("atlas_utils") + "/test/data/data.txt";
  ASSERT_EQ(result, expected_path);
}

TEST_F(TestGeneral, GetFullPath_NonExistentPathWithPrefix)
{
  EXPECT_THROW(atlas_utils::utils::GetFullPath("package://nonexistent/test/data/data.txt"), ament_index_cpp::PackageNotFoundError);
}

TEST_F(TestGeneral, GetFullPath_PathWithoutPrefix)
{
  std::string result = atlas_utils::utils::GetFullPath("hola k ase");
  std::string expected_path = "hola k ase";
  ASSERT_EQ(result, expected_path);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
