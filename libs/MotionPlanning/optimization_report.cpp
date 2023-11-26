#include <optimization_report.h>

namespace mp
{

void OptimizationReport::save(const std::string& filepath) const
{
  std::ofstream file;
  file.open(filepath);

  file << "stepsPerPhase" << " " << stepsPerPhase << std::endl;
  file << "totalCost" << " " << totalCost << std::endl;
  file << "qdim" << " " << slices.front().q.size() << std::endl;

  // trajectory
  file << std::endl;
  file << "trajectoryTree" << std::endl;
  for(std::size_t s{0}; s < slices.size(); s++)
  {
    file << s << " ";
    const auto& slice = slices[s];
    for(auto v: slice.q)
    {
       file << v << " ";
    }
    file << std::endl;
  }

  // objectives
  file << std::endl;
  for(const auto& o: objectives)
  {
    file << "objective" << " " << o.first << " " << o.second << std::endl;

    for(std::size_t s{0}; s < slices.size(); s++)
    {
      const auto& slice = slices[s];
      const auto ot = slice.objectivesResults.find(o.first);
      if(ot != slice.objectivesResults.cend())
      {
        file << s << " " << ot->second << std::endl;
      }
    }

    file << std::endl;
  }

  // irrelevant objectives
  file << "objectives_irrelevant_for_total_cost" << std::endl;
  for(const auto& o: objectivesIrrelevantForCost)
  {
    file << o << std::endl;
  }
  file << std::endl;

  // vars
  for(std::size_t b{0}; b < allVars.size(); b++)
  {
    file << "branch " << b << std::endl;

    const auto& var = allVars[b];

    for(const auto s: var.order0)
    {
      file << s << std::endl;
    }

    file << std::endl;
  }

  file.close();
}
}
