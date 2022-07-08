//
//  TetraApp.cpp
//
//  Created by srm, March 2020
//
#include "RastApp.hpp"
#include <nanogui/window.h>
#include <nanogui/glcanvas.h>
#include <nanogui/layout.h>

#include <cpplocate/cpplocate.h>

// Fixed screen size is awfully convenient, but you can also
// call Screen::setSize to set the size after the Screen base
// class is constructed.
const int RastApp::windowWidth = 800;
const int RastApp::windowHeight = 600;

bool useDeferred = true;
int bufferNum = 0;

// Constructor runs after nanogui is initialized and the OpenGL context is current.
RastApp::RastApp(string fileName)
    : nanogui::Screen(Eigen::Vector2i(windowWidth, windowHeight), "Our Rasterizer", false),
      backgroundColor(0.0f, 0.0f, 0.0f, 0.0f)
{

  const std::string resourcePath =
      cpplocate::locatePath("resources/Common", "", nullptr) + "resources/";

  // Set up a simple shader program by passing the shader filenames to the convenience constructor
  forwardShader.reset(new GLWrap::Program("Forward Shader", {{GL_VERTEX_SHADER, resourcePath + "../Rast/shaders/vert.vs"},
                                                             {GL_FRAGMENT_SHADER, resourcePath + "../Rast/shaders/frag.fs"}}));

  this->scene = Generator::generateScene(fileName);

  // Create a camera in a default position, respecting the aspect ratio of the window.
  cam = make_shared<RTUtil::PerspectiveCamera>(scene->cam);
  cc.reset(new RTUtil::DefaultCC(cam));
  float as = this->scene->cam.getAspectRatio();
  Screen::setSize(Eigen::Vector2i(windowWidth, windowWidth / as));

  // Set viewport
  Eigen::Vector2i framebufferSize;
  glfwGetFramebufferSize(glfwWindow(), &framebufferSize.x(), &framebufferSize.y());
  glViewport(0, 0, framebufferSize.x(), framebufferSize.y());

  // backgroundColor = nanogui::Color(scene->bgColor.x(), scene->bgColor.y(), scene->bgColor.z(), 0.0);

  // NanoGUI boilerplate
  performLayout();
  setVisible(true);

  for (pair<string, shared_ptr<NodeAnimation> > t : scene->keyframes)
  {
    std::cout << t.first << " "
              << t.second->getTransformation(1) << "\n";
  }
}

bool RastApp::keyboardEvent(int key, int scancode, int action, int modifiers)
{
  if (Screen::keyboardEvent(key, scancode, action, modifiers))
    return true;

  if (action == GLFW_PRESS)
  {
    switch (key)
    {
    case GLFW_KEY_ESCAPE:
      setVisible(false);
      return true;
    case GLFW_KEY_R:
      t = 0;
      return true;
    case GLFW_KEY_P:
      speed = 0;
      return true;
    case GLFW_KEY_LEFT:
      speed += 250;
      return true;
    case GLFW_KEY_RIGHT:
      speed -= 250;
      return true;
    default:
      return true;
    }
  }
  return cc->keyboardEvent(key, scancode, action, modifiers);
}

bool RastApp::mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers)
{
  return Screen::mouseButtonEvent(p, button, down, modifiers) ||
         cc->mouseButtonEvent(p, button, down, modifiers);
}

bool RastApp::mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers)
{
  return Screen::mouseMotionEvent(p, rel, button, modifiers) ||
         cc->mouseMotionEvent(p, rel, button, modifiers);
}

bool RastApp::scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel)
{
  return Screen::scrollEvent(p, rel) ||
         cc->scrollEvent(p, rel);
}

void RastApp::findLightsForward(shared_ptr<Node> node, Eigen::Affine3f acc)
{
  acc = acc * node->transform;

  for (int i = 0; i < node->lightInfos.size(); i++)
  {
    if (node->lightInfos[i]->type == RTUtil::Point)
    {
      Eigen::Vector3f power = node->lightInfos[i]->power;
      Eigen::Vector3f pos = acc * node->lightInfos[i]->position;
      forwardShader->uniform("lightPos", pos);
      forwardShader->uniform("power", power);
      return;
    }
  }

  for (int i = 0; i < node->children.size(); i++)
  {
    findLightsForward(node->children[i], acc);
  }
}

void RastApp::calculateBoneTransforms(shared_ptr<Node> node, Eigen::Affine3f acc, shared_ptr<MeshInfo> meshI, float t)
{
  Eigen::Matrix4f M_anim;
  try
  {
    M_anim = scene->keyframes.at(node->name)->getTransformation(t);
  }
  catch (...)
  {
    M_anim = node->transform.matrix();
  }
  acc = acc * M_anim;

  if (meshI->bindPoses.count(node->name) != 0)
  { // there IS a bone
    int boneId = meshI->bindPoses.at(node->name).first;
    Eigen::Affine3f invBind = meshI->bindPoses.at(node->name).second;
    boneTransforms[boneId] = (acc * invBind).matrix();
  }

  for (int i = 0; i < node->children.size(); i++)
  {
    calculateBoneTransforms(node->children[i], acc, meshI, t);
  }
}

void RastApp::traverseDrawForward(shared_ptr<Node> node, Eigen::Affine3f acc)
{
  currentTime = glfwGetTime();
  float deltaT = previousTime - currentTime;
  previousTime = currentTime;
  t += speed * deltaT;

  Eigen::Matrix4f M_anim;
  try
  {
    M_anim = scene->keyframes.at(node->name)->getTransformation(t);
  }
  catch (...)
  {
    M_anim = node->transform.matrix();
  }

  acc = acc * M_anim;
  forwardShader->uniform("mM", M_anim);
  // cout << M << "\n";
  // shared_ptr<NodeAnimation> anim = scene->keyframes.at(0);

  for (int i = 0; i < node->meshInfos.size(); i++)
  {
    shared_ptr<nori::BSDF> mat = node->meshInfos[i]->material;
    shared_ptr<nori::Microfacet> micro = dynamic_pointer_cast<nori::Microfacet>(mat);

    float alpha = micro->alpha();
    float eta = micro->eta();
    Eigen::Vector3f kd = micro->diffuseReflectance();
    forwardShader->uniform("alpha", alpha);
    forwardShader->uniform("eta", eta);
    forwardShader->uniform("k_d", kd);

    forwardShader->uniform("hasBones", node->meshInfos[i]->hasBones);

    if (node->meshInfos[i]->hasBones)
    {
      calculateBoneTransforms(scene->root, Eigen::Affine3f::Identity(), node->meshInfos[i], t);
      for (int i = 0; i < 50; i++)
      {
        forwardShader->uniform("boneTransforms[" + to_string(i) + "]", boneTransforms[i]);
      }
    }

    // forwardShader->uniform("theBoneTransform", boneTransforms[1]);
    // std::cout << boneTransforms[0] << "\n";

    node->meshInfos[i]->mesh->drawElements();
  }

  for (int i = 0; i < node->children.size(); i++)
  {
    traverseDrawForward(node->children[i], acc);
  }
}

void RastApp::drawContents()
{
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
  {
    std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
  }

  GLWrap::checkGLError("drawContents start");
  glClearColor(backgroundColor.r(), backgroundColor.g(), backgroundColor.b(), backgroundColor.w());
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);

  forwardShader->use();
  forwardShader->uniform("mV", cam->getViewMatrix().matrix());
  forwardShader->uniform("mP", cam->getProjectionMatrix().matrix());
  forwardShader->uniform("lightPos", Eigen::Vector3f(2., 2., 2.));
  forwardShader->uniform("power", Eigen::Vector3f(500., 500., 500.));

  findLightsForward(scene->root, Eigen::Affine3f::Identity());
  traverseDrawForward(scene->root, Eigen::Affine3f::Identity());
  forwardShader->unuse();
}
