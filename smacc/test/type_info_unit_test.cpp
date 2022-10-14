// Bring in my package's API, which is what I'm testing
#include <smacc/introspection/string_type_walker.h>
// Bring in gtest
#include <gtest/gtest.h>

using namespace smacc::introspection;

// Declare a test
TEST(TestSuite, testCase2)
{
  auto t = TypeInfo::getTypeInfoFromString("u0<u1,u2,u3,u4,u5,u6,u7,u8,u9>");

  ASSERT_EQ(t->templateParameters.size(), 9);
}

TEST(TestSuite, testCase1)
{
  std::stringstream buffer;

  // Save cout's buffer here
  std::streambuf *sbuf = std::cout.rdbuf();

  // Redirect cout to our stringstream buffer or any other ostream
  std::cout.rdbuf(buffer.rdbuf());
  // Use cout as usual
  std::cout.rdbuf(sbuf);

  auto t = TypeInfo::getTypeInfoFromString("u0<u1,u2,u3,u4,u5,u6,u7,u8,u9,u10, u11<u12,u13>>");

  ASSERT_EQ(t->templateParameters.size(), 11);

  // When done redirect cout to its old self

  ASSERT_EQ(t->templateParameters[10]->templateParameters.size(), 2);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  //ros::init(argc, argv, "tester");
  //ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
