#include "Mesh.h"

// basic file operations
#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

void Mesh::load( const char* filename )
{
	// 2.1.1. load() should populate bindVertices, currentVertices, and faces

	// Add your code here.
  ifstream obj (filename);

  cout << "Opening: " << filename << "... ";

  if (!obj)  // file fails to open
  {
    cerr << "failed to open " << filename << "... STOP!\n";
    exit(1);  // syscall to exit
  }
  else  // succeeeded
  {
    cout << "Success!\nNow connecting model... ";

    char type;      // vertices (v) or face (f)
    float x,y,z;   // in stream x,y,z position
    int a,b,c;    // in stream vertex a,b,c
    while (obj >> type)  // read until stream is empty
    {
      if (type == 'v')  // add vertex
      {
        obj >> x;
        obj >> y;
        obj >> z;
        Vector3f vert (x,y,z);
        bindVertices.push_back(vert);
      }
      else if (type == 'f')
      {
        obj >> a;
        obj >> b;
        obj >> c;
        Tuple3u face (a-1,b-1,c-1);
        faces.push_back(face);
      }
    }
    
    cout << "Connected the vertex!\nClosing...\n\n";
    obj.close();
  }

	// make a copy of the bind vertices as the current vertices
	currentVertices = bindVertices;
}

void Mesh::draw()
{
	// Since these meshes don't have normals
	// be sure to generate a normal per triangle.
	// Notice that since we have per-triangle normals
	// rather than the analytical normals from
	// assignment 1, the appearance is "faceted".

  int faceCount = faces.size();
  int vertCount = currentVertices.size();

  Tuple3u face;        // current face
  Vector3f vert;      // Vertecies associated to the current face (for loop)
  Vector3f d1,d2,n;  // normal vector

  glBegin(GL_TRIANGLES);

  for (int i = 0; i < faceCount; ++i)  // check all faces
  {
    face = faces[i];

    // render normal
    d1 = currentVertices[face[1]] - currentVertices[face[0]];
    d2 = currentVertices[face[2]] - currentVertices[face[1]];
    n = Vector3f::cross(d1,d2).normalized();

    glNormal3f(n[0],n[1],n[2]);

    // render triangle
    for (int v = 0; v < 3; ++v)  // for each vertices in the face
    {
      vert = currentVertices[face[v]];
      glVertex3f(vert[0],vert[1],vert[2]);
    }
  }

  glEnd();
}

void Mesh::loadAttachments( const char* filename, int numJoints )
{
	// 2.2. Implement this method to load the per-vertex attachment weights
	// this method should update m_mesh.attachments

  ifstream attach (filename);

  cout << "Opening: " << filename << " with " << numJoints << " joints ... ";

  if (!attach)  // file fails to open
  {
    cerr << "failed to open " << filename << "... STOP!\n";
    exit(1);  // syscall to exit
  }
  else  // succeeeded
  {
    cout << "Success!\nNow attaching model to joints... ";

    vector<float> attachList;  // create a vertex attachment
    float check;              // check if file is at the end or not
    //attachList.push_back(0.0f);

    while (attach >> check)  // read until stream is empty
    {
      attachList.push_back(check);

      for (int i = 2; i < numJoints; ++i)  // put rest of the valus into attach
      {
        attach >> check;
        attachList.push_back(check);
      }

      attachments.push_back(attachList);  // push list into attachments

      // cleanup attachList for next line
      attachList.clear();
      attachList.swap(std::vector<float>(attachList));
      //attachList.push_back(0.0f);
    }

    cout << "Attached the vertex to joints!\nClosing...\n\n";
    attach.close();
  }

}
