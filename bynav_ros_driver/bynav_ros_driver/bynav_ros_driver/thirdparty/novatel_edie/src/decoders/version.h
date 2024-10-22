#include <string>
#include <string.h>

#define RELEASE_VERSION "1.0.0"
#define GIT_SHA "d82103e6018b27f1519020d0ec5b26534221db08"
#define GIT_BRANCH "humble"
#define GIT_IS_DIRTY true
#define BUILD_TIMESTAMP "2024-04-11T09:23:48"

char* caPrettyPrint = NULL;

char* get_version()
{
   return RELEASE_VERSION;
}

char* get_git_sha()
{
   return GIT_SHA;
}

char* get_git_branch()
{
   return GIT_BRANCH;
}

bool git_git_is_dirty()
{
   return GIT_IS_DIRTY;
}

char* get_build_timestamp()
{
   return BUILD_TIMESTAMP;
}

char* get_pretty_version()
{
   if (!caPrettyPrint)
   {
      std::string sPretty;
      sPretty.append("Version: ") += get_version();
      if (GIT_IS_DIRTY)
         sPretty += "-Dirty";
      sPretty += "\n";
      sPretty.append("Branch: ") += get_git_branch();
      sPretty.append("\nSHA: ") += get_git_sha();

      caPrettyPrint = new char[sPretty.size()+1];
      strcpy(caPrettyPrint, sPretty.c_str());
   }
   return caPrettyPrint;
}
