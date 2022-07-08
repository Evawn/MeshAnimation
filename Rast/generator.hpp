#include <RTUtil/Camera.hpp>
#include <RTUtil/sceneinfo.hpp>
#include <RTUtil/conversions.hpp>
#include <Eigen/Core>
#include <GLWrap/Mesh.hpp>

#include <ext/assimp/include/assimp/Importer.hpp>  // Plain-C interface
#include <ext/assimp/include/assimp/scene.h>       // Output data structure
#include <ext/assimp/include/assimp/postprocess.h> // Post processing flags

using namespace std;

class NodeAnimation
{
public:
  NodeAnimation();
  unique_ptr<vector<pair<double, Eigen::Vector3f> > > scales;
  unique_ptr<vector<pair<double, Eigen::Quaternionf> > > rotations;
  unique_ptr<vector<pair<double, Eigen::Vector3f> > > translations;

  Eigen::Vector3f getScale(double t);
  Eigen::Quaternionf getRotation(double t);
  Eigen::Vector3f getTranslation(double t);

  Eigen::Matrix4f getTransformation(double t);
};

class vertexWeight
{
public:
  vertexWeight()
  {
    bones = vector<pair<int, float> >();
  }
  vector<pair<int, float> > bones;
};

class MeshInfo
{
public:
  shared_ptr<GLWrap::Mesh> mesh;
  shared_ptr<nori::BSDF> material;
  map<string, pair<int, Eigen::Affine3f> > bindPoses;
  /* bindPoses[node name] = (bone index, inverse bind pose matrix) */
  map<int, vertexWeight *> weights;
  bool hasBones;
  /* weights[vertex index]->bones[0] = (bone index, weight) of the first bone */
};

class Node
{
public:
  Node();
  string name;

  Eigen::Affine3f transform;
  vector<shared_ptr<MeshInfo> > meshInfos;
  vector<shared_ptr<RTUtil::LightInfo> > lightInfos;
  vector<shared_ptr<Node> > children;
  shared_ptr<Node> parent;
};

class Scene
{
public:
  Scene();

  RTUtil::PerspectiveCamera cam;
  shared_ptr<Node> root;

  RTUtil::SceneInfo info;
  shared_ptr<nori::BSDF> default_mat;
  Eigen::Vector3f bgColor;

  map<string, shared_ptr<NodeAnimation> > keyframes;
};

class Generator
{
  static void initializeAnimation(shared_ptr<Scene> scene, const aiScene *data);
  static void initializeScene(shared_ptr<Scene> scene, const aiScene *data);
  static void traverseNodes(shared_ptr<Scene> scene, const aiScene *data, shared_ptr<Node> parent, shared_ptr<Node> myNode, aiNode *node);

  static void initializeCamera(shared_ptr<Scene> scene, const aiScene *data);

public:
  static shared_ptr<Scene> generateScene(const string &filename);
};