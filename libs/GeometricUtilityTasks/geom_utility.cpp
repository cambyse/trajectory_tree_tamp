#include <geom_utility.h>

using namespace std;

uint getFrameIndex(const rai::KinematicWorld& G, const std::string& object_name)
{
  const auto object = G.getFrameByName( object_name.c_str() );
  for(auto i = 0; i < G.frames.size(); ++i )
  {
    if(object == G.frames(i))
    {
      return i;
    }
  }

  throw("object not found!");
}
