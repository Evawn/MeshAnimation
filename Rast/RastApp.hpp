//
//  TetraApp.hpp
//  Demo of basic usage of nanogui and GLWrap to get a simple object on the screen.
//
//  Created by srm, March 2020
//

#pragma once

#include <nanogui/screen.h>

#include <GLWrap/Program.hpp>
#include <GLWrap/Mesh.hpp>
#include <GLWrap/Framebuffer.hpp>
#include <RTUtil/Camera.hpp>
#include <RTUtil/CameraController.hpp>
#include <RTUtil/microfacet.hpp>
#include <RTUtil/Sky.hpp>

#include <generator.hpp>

using namespace std;

class RastApp : public nanogui::Screen
{
public:
  RastApp(string fileName);

  virtual bool keyboardEvent(int key, int scancode, int action, int modifiers) override;
  virtual bool mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers) override;
  virtual bool mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers) override;
  virtual bool scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel) override;

  virtual void drawContents() override;

private:
  void traverseDrawForward(shared_ptr<Node> node, Eigen::Affine3f acc);
  void findLightsForward(shared_ptr<Node> node, Eigen::Affine3f acc);
  void calculateBoneTransforms(shared_ptr<Node> node, Eigen::Affine3f acc, shared_ptr<MeshInfo> meshI, float t);

  static const int windowWidth;
  static const int windowHeight;

  Eigen::Matrix4f boneTransforms[50];

  std::unique_ptr<GLWrap::Program> forwardShader;

  std::shared_ptr<Scene> scene;

  std::shared_ptr<RTUtil::PerspectiveCamera> cam;
  std::unique_ptr<RTUtil::DefaultCC> cc;

  nanogui::Color backgroundColor;

  std::unique_ptr<GLWrap::Mesh> fsqMesh;

  float previousTime = 0; // clock time
  float currentTime = 0;  // clock time
  float speed = 1000;     // speed
  float t = 0;            // animation time
  bool rewind = false;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
