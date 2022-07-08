#include "generator.hpp"

using namespace std;

string resourcePath = "../resources/scenes/";
string infoFile = "_info.json";
string meshFile = ".dae";

Node::Node()
{
  this->transform = Eigen::Affine3f::Identity();
  this->children = vector<shared_ptr<Node> >();
}

Scene::Scene() : cam(
                     Eigen::Vector3f(6, 8, 10), // eye
                     Eigen::Vector3f(0, 0, 0),  // target
                     Eigen::Vector3f(0, 1, 0),  // up
                     4. / 3.,                   // aspect
                     0.1, 50.0,                 // near, far
                     15.0 * M_PI / 180          // fov
                 ){};

NodeAnimation::NodeAnimation()
{
  scales.reset(new vector<pair<double, Eigen::Vector3f> >());
  rotations.reset(new vector<pair<double, Eigen::Quaternionf> >());
  translations.reset(new vector<pair<double, Eigen::Vector3f> >());
}

Eigen::Vector3f NodeAnimation::getScale(double t)
{
  if (this->scales->size() != 0)

  {
    pair<double, Eigen::Vector3f> prev = this->scales->at(0);
    pair<double, Eigen::Vector3f> next = this->scales->at(0);
    int i = 0;
    bool same = true;

    while (next.first < t && i < this->scales->size() + 1)
    {
      prev = next;
      same = true;
      if (i < this->scales->size())
      {
        next = this->scales->at(i);
        same = false;
      }
      i++;
    }

    return same ? prev.second : (prev.second * ((next.first - t) / (next.first - prev.first))) + (next.second * ((t - prev.first) / (next.first - prev.first)));
  }
  else
  {
    return Eigen::Vector3f(1, 1, 1);
  }
}

Eigen::Quaternionf NodeAnimation::getRotation(double t)
{
  if (this->rotations->size() != 0)

  {
    pair<double, Eigen::Quaternionf> prev = this->rotations->at(0);
    pair<double, Eigen::Quaternionf> next = this->rotations->at(0);
    int i = 0;
    bool same = true;

    while (next.first < t && i < this->rotations->size() + 1)
    {
      prev = next;
      same = true;
      if (i < this->rotations->size())
      {
        next = this->rotations->at(i);
        same = false;
      }
      i++;
    }

    return same ? prev.second : prev.second.slerp((t - prev.first) / (next.first - prev.first), next.second);
  }
  else
  {
    return Eigen::Quaternionf::Identity();
  }
}

Eigen::Vector3f NodeAnimation::getTranslation(double t)
{
  if (this->translations->size() != 0)

  {
    pair<double, Eigen::Vector3f> prev = this->translations->at(0);
    pair<double, Eigen::Vector3f> next = this->translations->at(0);
    int i = 0;
    bool same = true;

    while (next.first < t && i < this->translations->size() + 1)
    {
      prev = next;
      same = true;
      if (i < this->translations->size())
      {
        next = this->translations->at(i);
        same = false;
      }
      i++;
    }

    return same ? prev.second : (prev.second * ((next.first - t) / (next.first - prev.first))) + (next.second * ((t - prev.first) / (next.first - prev.first)));
  }
  else
  {

    return Eigen::Vector3f(0, 0, 0);
  }
}

Eigen::Matrix4f NodeAnimation::getTransformation(double t)
{
  Eigen::Vector3f translation = getTranslation(t);
  Eigen::Matrix4f T;
  T << 1, 0, 0, translation.x(),
      0, 1, 0, translation.y(),
      0, 0, 1, translation.z(),
      0, 0, 0, 1;

  Eigen::Vector3f scaleLin = getScale(t);
  Eigen::Vector4f scale(scaleLin.x(), scaleLin.y(), scaleLin.z(), 1);
  Eigen::Matrix4f S = scale.asDiagonal();

  Eigen::Quaternionf rotation = getRotation(t);
  Eigen::Matrix4f R;
  R.setIdentity();
  R.block<3, 3>(0, 0) = rotation.toRotationMatrix();

  // cout << "Rotation: " << R << "\n";
  // cout << "Scale: " << S << "\n";
  // cout << "Translation: " << T << "\n \n";

  return T * R * S;
}

void Generator::initializeAnimation(shared_ptr<Scene> scene, const aiScene *data)
{
  if (data->HasAnimations())
  {
    aiAnimation *anim = data->mAnimations[0];
    // cout << "Num Animations: " << data->mNumAnimations << "\n";
    // cout << "Num channels of first animation: " << anim->mNumChannels << "\n";
    for (int i = 0; i < anim->mNumChannels; i++)
    {
      aiNodeAnim *anodeAnim = anim->mChannels[i];
      string name = anodeAnim->mNodeName.C_Str();

      shared_ptr<NodeAnimation> myNodeAnim = make_shared<NodeAnimation>();
      // (1) make instance of nodeAnimation

      // Retrieve Scale Keyframes
      for (int j = 0; j < anodeAnim->mNumScalingKeys; j++)
      {
        double time = anodeAnim->mScalingKeys[j].mTime;
        Eigen::Vector3f scale = RTUtil::a2e(anodeAnim->mScalingKeys[j].mValue);
        myNodeAnim->scales->push_back(make_pair(time, scale));
        //cout << "scale!\n";
      }

      //Retrieve Rotation Keyframes
      for (int j = 0; j < anodeAnim->mNumRotationKeys; j++)
      {
        double time = anodeAnim->mScalingKeys[j].mTime;
        Eigen::Quaternionf rotation = RTUtil::a2e(anodeAnim->mRotationKeys[j].mValue);
        myNodeAnim->rotations->push_back(make_pair(time, rotation));
        //cout << "rotato!\n";
      }

      // Retrieve Translation Keyframes
      for (int j = 0; j < anodeAnim->mNumPositionKeys; j++)
      {
        double time = anodeAnim->mScalingKeys[j].mTime;
        Eigen::Vector3f position = RTUtil::a2e(anodeAnim->mPositionKeys[j].mValue);
        myNodeAnim->translations->push_back(make_pair(time, position));
        //cout << "translato!\n";
      }
      // (2) populate scale, rotation, translation vectors

      //cout << "node animation name: " << name << "\n";
      scene->keyframes.insert(make_pair(name, myNodeAnim));

      // (3) add node animation to the map
    }
  }
}

void Generator::initializeScene(shared_ptr<Scene> scene, const aiScene *data)
{
  cout << "Initializing scene...\n";
  aiNode *node = data->mRootNode;
  scene->root = make_shared<Node>();
  scene->keyframes = map<string, shared_ptr<NodeAnimation> >();
  traverseNodes(scene, data, nullptr, scene->root, node);
}

//initializes all info of a node, intializes children
void Generator::traverseNodes(shared_ptr<Scene> scene, const aiScene *data, shared_ptr<Node> parent, shared_ptr<Node> myNode, aiNode *aNode)
{
  cout << "Traversing nodes...\n";
  Eigen::Affine3f T = RTUtil::a2e(aNode->mTransformation);
  myNode->name = aNode->mName.C_Str();
  myNode->transform = T;
  myNode->parent = parent;

  cout << "Node named: " << myNode->name << "\n";

  // for each mesh...
  for (int m = 0; m < aNode->mNumMeshes; m++)
  {
    cout << "Mesh " << m << "\n";
    // make a mesh

    cout << "Mesh " << m << "\n";

    //-----
    unsigned int meshNum = aNode->mMeshes[m];
    aiMesh *mesh = data->mMeshes[meshNum];

    int nVerts = mesh->mNumVertices;
    int nFaces = mesh->mNumFaces;
    //-----

    // add verts to mesh
    Eigen::Matrix<float, 3, Eigen::Dynamic> positions(3, nVerts);
    for (int i = 0; i < nVerts; i++)
    {
      Eigen::Vector3f vert = RTUtil::a2e(mesh->mVertices[i]);
      positions.col(i) << vert.x(), vert.y(), vert.z();
    }

    shared_ptr<GLWrap::Mesh> curr_mesh = make_shared<GLWrap::Mesh>();
    curr_mesh->setAttribute(0, positions);

    // add triangles to mesh
    Eigen::VectorXi indices(3 * nFaces);
    for (int i = 0; i < nFaces; i++)
    {
      indices(3 * i) = mesh->mFaces[i].mIndices[0];
      indices(3 * i + 1) = mesh->mFaces[i].mIndices[1];
      indices(3 * i + 2) = mesh->mFaces[i].mIndices[2];
    }
    curr_mesh->setIndices(indices, GL_TRIANGLES);

    // add normals to mesh
    Eigen::Matrix<float, 3, Eigen::Dynamic> normals(3, nVerts);
    for (int i = 0; i < nVerts; i++)
    {
      Eigen::Vector3f normal = RTUtil::a2e(mesh->mNormals[i]);
      normals.col(i) << normal.x(), normal.y(), normal.z();
    }
    curr_mesh->setAttribute(1, normals);

    // Add the material for the geometry
    // If the mesh has a material and there is a BSDF in the scene info with a "name" field matching the material name, use it.
    // Otherwise, if the mesh is below a node whose name matches the "node" field of a material in the scene info, use it.
    // Otherwise use the scene's default material.

    // add the mesh to the node
    shared_ptr<MeshInfo> meshi = make_shared<MeshInfo>();
    meshi->mesh = curr_mesh;

    aiString meshMat = data->mMaterials[mesh->mMaterialIndex]->GetName();
    try
    {
      std::shared_ptr<nori::BSDF> mat = scene->info.namedMaterials.at(meshMat.C_Str());
      meshi->material = mat;
    }
    catch (...)
    {
      try
      {
        std::shared_ptr<nori::BSDF> mat = scene->info.nodeMaterials.at(aNode->mName.C_Str());
        meshi->material = mat;
      }
      catch (...)
      {
        std::shared_ptr<nori::BSDF> mat = scene->info.defaultMaterial;
        meshi->material = mat;
      }
    }

    // DO BONE STUFF!
    meshi->hasBones = mesh->HasBones();
    if (mesh->HasBones())
    {
      for (int i = 0; i < mesh->mNumBones; i++)
      {
        aiBone *daBone = mesh->mBones[i];

        // STORE BIND POSE -----
        string boneName = daBone->mName.C_Str();
        Eigen::Affine3f invBind = RTUtil::a2e(daBone->mOffsetMatrix);
        meshi->bindPoses.insert(make_pair(boneName, make_pair(i, invBind)));

        // STORE WEIGHTS -----
        for (int j = 0; j < daBone->mNumWeights; j++)
        {
          aiVertexWeight vWeight = daBone->mWeights[j];
          unsigned int vID = vWeight.mVertexId;
          float w = vWeight.mWeight;

          if (meshi->weights.count(vID) == 0)
          { // key hasn't been created before
            meshi->weights.insert(make_pair(vID, new vertexWeight()));
          }
          meshi->weights[vID]->bones.push_back(make_pair(i, w));
        }
      }

      // add boneIDs to mesh
      Eigen::Matrix<float, 4, Eigen::Dynamic> boneIDs(4, nVerts);
      Eigen::Matrix<float, 4, Eigen::Dynamic> boneWeights(4, nVerts);
      for (int i = 0; i < nVerts; i++)
      {
        vector<pair<int, float> > boneInfo = meshi->weights[i]->bones;
        for (int j = 0; j < 4; j++)
        {
          if (j >= boneInfo.size())
          {
            boneIDs(j, i) = 0;
            boneWeights(j, i) = 0;
          }
          else
          {
            pair<int, float> IdWeight = boneInfo[j];
            boneIDs(j, i) = IdWeight.first;
            boneWeights(j, i) = IdWeight.second;
          }
        }
      }
      meshi->mesh->setAttribute(2, boneIDs);
      meshi->mesh->setAttribute(3, boneWeights);
    }

    myNode->meshInfos.push_back(meshi);
  }

  // Parse through the scene info
  // Look for lights under node
  // for (int i = 0; i < scene->info.lights.size(); i++)
  // {
  //   std::shared_ptr<RTUtil::LightInfo> l = scene->info.lights[i];

  //   if (aNode->mName.C_Str() == l->nodeName)
  //   {
  //     if (l->type == RTUtil::Point)
  //     {
  //       myNode->lightInfos.push_back(l);
  //       std::cout << "Added a point light \n";
  //     }
  //   }
  // }

  // Recurse through the children nodes
  for (int c = 0; c < aNode->mNumChildren; c++)
  {
    shared_ptr<Node> newNode = make_shared<Node>();
    // weak_ptr<Node> weakyboi = newNode;
    myNode->children.push_back(newNode);
    traverseNodes(
        scene,
        data,
        myNode,
        newNode,
        aNode->mChildren[c]);
  }
  cout << "Children: " << myNode->children.size() << "\n";
  cout << "Meshes: " << myNode->meshInfos.size() << "\n";
}

void Generator::initializeCamera(shared_ptr<Scene> scene, const aiScene *data)
{
  if (data->HasCameras())
  {

    aiCamera *camData = data->mCameras[0];
    aiString camNodeName = camData->mName;

    aiNode *curNode = data->mRootNode->FindNode(camNodeName);

    //Eigen::IOFormat format(Eigen::StreamPrecision, 0, ", ", "; ", "", "", "[", "]\n");

    Eigen::Affine3f M = Eigen::Affine3f::Identity();
    while (curNode != NULL)
    {
      M = RTUtil::a2e(curNode->mTransformation) * M;
      curNode = curNode->mParent;
    }

    Eigen::Vector3f camPosWorld = M * RTUtil::a2e(camData->mPosition);
    Eigen::Vector3f camLookAtWorld = M.linear() * RTUtil::a2e(camData->mLookAt);
    Eigen::Vector3f camUpWorld = M.linear() * RTUtil::a2e(camData->mUp);

    float t = (-1 * camPosWorld).dot(camLookAtWorld) / camLookAtWorld.dot(camLookAtWorld) / 3;
    Eigen::Vector3f tar = camPosWorld + camLookAtWorld * 10.; // * t;

    float vfov = 2 * tan(atan(camData->mHorizontalFOV / 2) / camData->mAspect);

    RTUtil::PerspectiveCamera camera = RTUtil::PerspectiveCamera(
        camPosWorld, // eye
        tar,
        camUpWorld,       // up
        camData->mAspect, // aspect
        camData->mClipPlaneNear,
        camData->mClipPlaneFar, // near, far
        vfov                    // fov
    );

    scene->cam = camera;
  }
}

shared_ptr<Scene> Generator::generateScene(const string &filename)
{
  string base = filename.substr(0, filename.find('.'));
  cout << base << '\n';

  // create pointer to scene
  shared_ptr<Scene> scene = make_shared<Scene>();
  // scene.reset(new Scene());

  // fill in info
  RTUtil::SceneInfo info;
  std::cout << RTUtil::readSceneInfo(resourcePath + base + infoFile, info);
  scene->info = info;

  // NEED TO ADD MATERIAL STUFF HERE
  scene->default_mat = info.defaultMaterial;

  // Import the scene files
  Assimp::Importer importer;
  importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, aiComponent_NORMALS);
  const aiScene *data = importer.ReadFile(
      resourcePath + filename,
      aiProcess_RemoveComponent |
          aiProcess_GenNormals |
          aiProcess_LimitBoneWeights |
          aiProcess_Triangulate |
          aiProcess_SortByPType);

  if (!data)
  {
    printf("Scene failed in import!\n");
  }
  else
  {
    printf("Scene imported successfully\n");
  }

  //INITIALIZE CAMERA
  //INITIALIZE SCENE
  initializeScene(scene, data);
  initializeCamera(scene, data);
  initializeAnimation(scene, data);

  // Add ambient lights
  for (int i = 0; i < scene->info.lights.size(); i++)
  {
    std::shared_ptr<RTUtil::LightInfo> l = scene->info.lights[i];

    if (l->type == RTUtil::Ambient)
    {
      scene->root->lightInfos.push_back(l);
      scene->bgColor = l->radiance;
      std::cout << "Added an ambient light \n";
    }
  }

  importer.FreeScene();
  return scene;
}