#include "SkeletalModel.h"

#include <FL/Fl.H>

// basic file operations
#include <iostream>
#include <fstream>

// string operation
#include <string.h>  // C
#include <string>   // C++

// math operation
#include <cmath>

// project header files
#include "Joint.h"

// defined values
#define B_THICKNESS 0.035f

using namespace std;

void SkeletalModel::load(const char *skeletonFile, const char *meshFile, const char *attachmentsFile)
{
	loadSkeleton(skeletonFile);

	m_mesh.load(meshFile);
	m_mesh.loadAttachments(attachmentsFile, m_joints.size());

	computeBindWorldToJointTransforms();
	updateCurrentJointToWorldTransforms();
}

void SkeletalModel::draw( Matrix4f cameraMatrix, bool skeletonVisible )
{
	// draw() gets called whenever a redraw is required
	// (after an update() occurs, when the camera moves, the window is resized, etc)

	m_matrixStack.clear();
	m_matrixStack.push(cameraMatrix);

	if( skeletonVisible )
	{
		drawJoints();

		drawSkeleton();
	}
	else
	{
		// Clear out any weird matrix we may have been using for drawing the bones and revert to the camera matrix.
		glLoadMatrixf(m_matrixStack.top());

		// Tell the mesh to draw itself.
		m_mesh.draw();
	}
}

void SkeletalModel::loadSkeleton( const char* filename )
{
	// Load the skeleton from file here.
  ifstream skel (filename);
  string line;

  cout << "Opening: " << filename << "... ";

  if (!skel)  // file fails to open
  {
    cerr << "failed to open " << filename << "... STOP!\n";
    exit(1);  // syscall to exit
  }
  else  // succeeeded
  {
    cout << "Success!\nNow connecting joints... ";
    
    float x,y,z;   // in stream x,y,z position
    int i;        // in stream root
    while (skel >> x)  // read until stream is empty
    {
      skel >> y;
      skel >> z;
      skel >> i;
      addJoint(x,y,z,i);
      /* Note: Not a great implementation but this will do for now */
    }
    
    cout << "Connected the joints!\nClosing...\n\n";
    skel.close();
  }
}

void SkeletalModel::addJoint( float x, float y, float z, int i )
{
  Joint *joint = new Joint();
  joint->transform = Matrix4f::translation(x,y,z);
  m_joints.push_back(joint);

  if (i == -1)  // if its the root joint
    m_rootJoint = joint;
  else  // add child to parent's list via parent's index
    m_joints[i]->children.push_back(joint);
}

void SkeletalModel::drawJoints( )
{
	// Draw a sphere at each joint. You will need to add a recursive helper function to traverse the joint hierarchy.
	//
	// We recommend using glutSolidSphere( 0.025f, 12, 12 )
	// to draw a sphere of reasonable size.
	//
	// You are *not* permitted to use the OpenGL matrix stack commands
	// (glPushMatrix, glPopMatrix, glMultMatrix).
	// You should use your MatrixStack class
	// and use glLoadMatrix() before your drawing call.

  drawChildJoint(m_rootJoint);
}

void SkeletalModel::drawChildJoint( Joint* child )
{
  // push translation to joint
  m_matrixStack.push(child->transform);
  glLoadMatrixf(m_matrixStack.top());

  // draw sphere
  glutSolidSphere(0.025f, 12, 12);

  // recurse with child (if its not empty)
  if (!child->children.empty())
    for (unsigned i = 0; i < child->children.size(); ++i)
      drawChildJoint(child->children[i]);

  // pop translation to parent or root
  m_matrixStack.pop();
  glLoadMatrixf(m_matrixStack.top());
}

void SkeletalModel::drawSkeleton( )
{
	// Draw boxes between the joints. You will need to add a recursive helper function to traverse the joint hierarchy.
  drawChildSkeleton(m_rootJoint);
}

void SkeletalModel::drawChildSkeleton( Joint* child )
{
  // setup reusable variables
  float len, ang;
  Vector3f aor;
  Matrix4f translate, scale, rotate, finMatrix;

	// draw if the joint has child joints
  if (!child->children.empty()){
    // push translation to joint
    m_matrixStack.push(child->transform);
    glLoadMatrixf(m_matrixStack.top());
    
    // look into the other child joint to connect bones and go deeper
    for (unsigned i = 0; i < child->children.size(); ++i)
    {
      // pre-process values
      len = length(child->children[i]);
        // my approach
        aor = axis(child->children[i]);
        ang = angle(child->children[i]);
      // calculate bone dimensions (matrix)
      translate = Matrix4f::translation(0.0f,0.0f,0.5);
      scale = Matrix4f::scaling(B_THICKNESS,B_THICKNESS,len);
        // my approach
        //rotate = Matrix4f::rotation(aor,ang);
        // intended approach
        rotate = rotateMatrix(child->children[i]);

      finMatrix = rotate * scale * translate;

      // push in the bone matrix
      m_matrixStack.push(finMatrix);
      glLoadMatrixf(m_matrixStack.top());
      
      // draw bone
      glutSolidCube(1.0f);

      // revert to joint matrix
      m_matrixStack.pop();
      glLoadMatrixf(m_matrixStack.top());

      // look into the child joint
      drawChildSkeleton(child->children[i]);
    }
    
    // pop translation
    m_matrixStack.pop();
    glLoadMatrixf(m_matrixStack.top());
  }
}

float SkeletalModel::length( Joint* joint )
{
  // get vector3f from last column from joint's transform
  Vector3f vector = joint->transform.getCol(3).xyz();
  // return the distance of the vector
  return vector.abs();
}

Vector3f SkeletalModel::axis( Joint* joint )
{
  // get vector3f from pointing vertex
  Vector3f pointing = -Vector3f::FORWARD;
  // get vector3f from last column from joint's transform
  Vector3f vector = joint->transform.getCol(3).xyz();
  // return the axis of rotation of the vectors
  return Vector3f::cross(pointing, vector);
}

float SkeletalModel::angle( Joint* joint )
{
  // get vector3f from pointing vertex
  Vector3f pointing = -Vector3f::FORWARD;
  // get vector3f from last column from joint's transform
  Vector3f vector = joint->transform.getCol(3).xyz();

  // compute dot product from two vector
  float dot = Vector3f::dot(pointing,vector);
  // compute product of magnitude
  float prod = pointing.abs() * vector.abs();

  // return the distance of the vector
  return (float)acos(dot/prod);
}

Matrix4f SkeletalModel::rotateMatrix( Joint* joint )
{
  Vector3f input = joint->transform.getCol(3).xyz();
  if (input == Vector3f::ZERO)
    return Matrix4f::identity();
  else
  {
    Vector3f r = -Vector3f::FORWARD;
    Vector3f z = joint->transform.getCol(3).xyz().normalized();
    Vector3f y = Vector3f::cross(z,r).normalized();
    Vector3f x = Vector3f::cross(y,z).normalized();

    Matrix4f rotate;

	  rotate.setCol( 0, Vector4f( x, 0 ));
	  rotate.setCol( 1, Vector4f( y, 0 ));
	  rotate.setCol( 2, Vector4f( z, 0 ));
	  rotate.setCol( 3, Vector4f( 0, 0, 0, 1 ) );

    return rotate;
  }
}

void SkeletalModel::setJointTransform(int jointIndex, float rX, float rY, float rZ)
{
	// Set the rotation part of the joint's transformation matrix based on the passed in Euler angles.
  Joint* indexJoint = m_joints[jointIndex];
  Vector3f pos = indexJoint->transform.getCol(3).xyz();
  Matrix4f rot = Matrix4f::rotateZ(rZ) * Matrix4f::rotateY(rY) * Matrix4f::rotateX(rX);

  Matrix4f total = Matrix4f::translation(pos) * rot;

  indexJoint->transform = total;
}


void SkeletalModel::computeBindWorldToJointTransforms()
{
	// 2.3.1. Implement this method to compute a per-joint transform from
	// world-space to joint space in the BIND POSE.
	//
	// Note that this needs to be computed only once since there is only
	// a single bind pose.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
  
  m_matrixStack.clear();
  computeChildBWTJTransforms(m_rootJoint);
}

void SkeletalModel::computeChildBWTJTransforms( Joint* child )
{
  // push
  m_matrixStack.push(child->transform);

  // compute bind from inverse
  child->bindWorldToJointTransform =  m_matrixStack.top();

  // recurse with child (if its not empty)
  if (!child->children.empty())
    for (unsigned i = 0; i < child->children.size(); ++i)
      computeChildBWTJTransforms(child->children[i]);

  // pop
  m_matrixStack.pop();
}

void SkeletalModel::updateCurrentJointToWorldTransforms()
{
	// 2.3.2. Implement this method to compute a per-joint transform from
	// joint space to world space in the CURRENT POSE.
	//
	// The current pose is defined by the rotations you've applied to the
	// joints and hence needs to be *updated* every time the joint angles change.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
  
  m_matrixStack.clear();
  updateChildCJTWTransforms(m_rootJoint);
}

void SkeletalModel::updateChildCJTWTransforms( Joint* child )
{
  // push
  m_matrixStack.push(child->transform);

  // compute current Joint to World Transform
  child->currentJointToWorldTransform =  m_matrixStack.top();

  // recurse with child (if its not empty)
  if (!child->children.empty())
    for (unsigned i = 0; i < child->children.size(); ++i)
      updateChildCJTWTransforms(child->children[i]);

  // pop
  m_matrixStack.pop();
}

void SkeletalModel::updateMesh()
{
	// 2.3.2. This is the core of SSD.
	// Implement this method to update the vertices of the mesh
	// given the current state of the skeleton.
	// You will need both the bind pose world --> joint transforms.
	// and the current joint --> world transforms.

  // calculate static value of vertices count and joint count
  int vertCount = m_mesh.bindVertices.size();
  int jointCount = m_joints.size();
  // allocate memory for these variables
  Vector3f newVertex;
  vector<float> currAttach;
  float weight;
  Matrix4f T,B;

  // compute all vertices
  for (int v = 0; v < vertCount; ++v)
  {
    // init zero vector and get attachment weights
    newVertex = Vector3f::ZERO;
    currAttach = m_mesh.attachments[v];

    // calculate vertex w/ weight from all joints
    for (int ji = 1; ji < jointCount; ++ji)
    {
      weight = currAttach[ji-1];
      T = m_joints[ji]->currentJointToWorldTransform;
      B = m_joints[ji]->bindWorldToJointTransform.inverse();

      // save process time if weight is 0
      if (weight != 0)
        newVertex += (weight * (T * B * Vector4f(m_mesh.bindVertices[v],1.0f))).xyz();
    }

    m_mesh.currentVertices[v] = newVertex;
  }
}

