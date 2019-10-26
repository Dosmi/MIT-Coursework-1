#include "surf.h"
#include "extra.h"
using namespace std;

namespace
{

    // We're only implenting swept surfaces where the profile curve is
    // flat on the xy-plane.  This is a check function.
    static bool checkFlat(const Curve &profile)
    {
        cerr << "profile size: " << profile.size() << endl;
        for (unsigned i=0; i<profile.size(); i++)
        { // if profile is empty, just returns true (why it works without any normals etc ...
          profile[i].V.print();
          profile[i].T.print();
          profile[i].N.print();

          cerr << i << ") V:" << profile[i].V[2] << " T:" << profile[i].T[2] << " N:" << profile[i].N[2] << endl;
          // cerr << "V1:" << profile[i].V[1] << " T1:" << profile[i].T[1] << " N1:" << profile[i].N[1] << endl;
          // cerr << "V0:" << profile[i].V[0] << " T0:" << profile[i].T[0] << " N0:" << profile[i].N[0] << endl;
          if (profile[i].V[2] != 0.0 ||
              profile[i].T[2] != 0.0 ||
              profile[i].N[2] != 0.0)
              {
                cerr << "returning false" << endl;
                return false;
              }

        }


        return true;
    }
}

/******************************************************************************************************

                   '||               .|'''.|                     .'|. '||''|.
.. .. ..    ....    ||  ..    ....   ||..  '  ... ...  ... ..  .||.    ||   ||    ....  .... ...
 || || ||  '' .||   || .'   .|...||   ''|||.   ||  ||   ||' ''  ||     ||''|'   .|...||  '|.  |
 || || ||  .|' ||   ||'|.   ||      .     '||  ||  ||   ||      ||     ||   |.  ||        '|.|
.|| || ||. '|..'|' .||. ||.  '|...' |'....|'   '|..'|. .||.    .||.   .||.  '|'  '|...'    '|


*******************************************************************************************************/

Surface makeSurfRev(const Curve &profile, unsigned steps)
{   // Surfaces of Revolution
    Surface surface;

    if (!checkFlat(profile))
    {
        cerr << "surfRev profile curve must be flat on xy plane." << endl;
        // exit(0);
    }
    // TODO: Here you should build the surface.  See surf.h for details.

    // +1. evaluate the control points along the profile curve (curve.cpp)
    /* 2. generate the vertices of the surface ...
          ... by simply duplicating the evaluated curve ...
          ... points at evenly sampled values of rotation. */

    cerr << profile.size() << endl;
    for( unsigned i = 0; i < profile.size(); i++ )
    {
      int surface_point_index = 0;
      cerr << "steps:" << steps << endl;
      for ( float singleRotation = (2*3.14)/steps; singleRotation <= 2*3.14; singleRotation += (2*3.14)/steps )
      {
        Matrix4f matrixOfOnes(1);
        // translate the 3x1 profile vector by the matrix of ones:
        // (just so we are able to apply transforms later by multiplying it with a 4x4 matrix)
        Matrix4f profileMatrix = matrixOfOnes.translation(profile[i].V);

        // this gets the necessary rotation matrix (on Y axis)
        Matrix4f rotationMatrix = matrixOfOnes.rotateY(singleRotation);
        // finally, this multiplication rotates the vector (represented by a matrix)
        Matrix4f rotatedYmatrix = rotationMatrix * profileMatrix;

        // get the last column (where the homogenious vector is)
        Vector4f rotatedHomogeniousVector = rotatedYmatrix.getCol(3);

        surface.VV.push_back( rotatedHomogeniousVector.xyz() );

        surface.VN.push_back( rotatedHomogeniousVector.yzx() );

        Tup3u triangle(1u,2u,3u);

        surface.VF.push_back(triangle);
        // cerr << "5" << endl;
        surface_point_index++;
      }

    }

    cerr << "\t>>> makeSurfRev called (but not implemented).\n\t>>> Returning empty surface." << endl;

    return surface;
}

/********************************************************************************************************

                   '||               ..|'''.|                     ..|'''.|          '||
.. .. ..    ....    ||  ..    ....  .|'     '    ....  .. ...   .|'     '  .... ...  ||
 || || ||  '' .||   || .'   .|...|| ||    .... .|...||  ||  ||  ||          '|.  |   ||
 || || ||  .|' ||   ||'|.   ||      '|.    ||  ||       ||  ||  '|.      .   '|.|    ||
.|| || ||. '|..'|' .||. ||.  '|...'  ''|...'|   '|...' .||. ||.  ''|....'     '|    .||.
                                                                           .. |
                                                                            ''
*********************************************************************************************************/

Surface makeGenCyl(const Curve &profile, const Curve &sweep )
{
    Surface surface;

    if (!checkFlat(profile))
    {
        cerr << "genCyl profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // TODO: Here you should build the surface.  See surf.h for details.

    cerr << "\t>>> makeGenCyl called (but not implemented).\n\t>>> Returning empty surface." <<endl;

    return surface;
}

/***********************************************************************************************************

     '||                               .|'''.|                     .'|.
   .. ||  ... ..   ....   ... ... ...  ||..  '  ... ...  ... ..  .||.    ....     ....    ....
 .'  '||   ||' '' '' .||   ||  ||  |    ''|||.   ||  ||   ||' ''  ||    '' .||  .|   '' .|...||
 |.   ||   ||     .|' ||    ||| |||   .     '||  ||  ||   ||      ||    .|' ||  ||      ||
 '|..'||. .||.    '|..'|'    |   |    |'....|'   '|..'|. .||.    .||.   '|..'|'  '|...'  '|...'


************************************************************************************************************/

void drawSurface(const Surface &surface, bool shaded)
{
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    if (shaded)
    {
        // This will use the current material color and light
        // positions.  Just set these in drawScene();
        glEnable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // This tells openGL to *not* draw backwards-facing triangles.
        // This is more efficient, and in addition it will help you
        // make sure that your triangles are drawn in the right order.
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
    }
    else
    {
        glDisable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        glColor4f(0.4f,0.4f,0.4f,1.f);
        glLineWidth(1);
    }

    glBegin(GL_TRIANGLES);
    for (unsigned i=0; i<surface.VF.size(); i++)
    {
        glNormal(surface.VN[surface.VF[i][0]]);
        glVertex(surface.VV[surface.VF[i][0]]);
        glNormal(surface.VN[surface.VF[i][1]]);
        glVertex(surface.VV[surface.VF[i][1]]);
        glNormal(surface.VN[surface.VF[i][2]]);
        glVertex(surface.VV[surface.VF[i][2]]);
    }
    glEnd();

    glPopAttrib();
}

/*****************************************************************************************************

     '||                              '|.   '|'                                    '||
   .. ||  ... ..   ....   ... ... ...  |'|   |    ...   ... ..  .. .. ..    ....    ||   ....
 .'  '||   ||' '' '' .||   ||  ||  |   | '|. |  .|  '|.  ||' ''  || || ||  '' .||   ||  ||. '
 |.   ||   ||     .|' ||    ||| |||    |   |||  ||   ||  ||      || || ||  .|' ||   ||  . '|..
 '|..'||. .||.    '|..'|'    |   |    .|.   '|   '|..|' .||.    .|| || ||. '|..'|' .||. |'..|'

*****************************************************************************************************/

void drawNormals(const Surface &surface, float len)
{
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_LIGHTING);
    glColor4f(0,1,1,1);
    glLineWidth(1);

    glBegin(GL_LINES);
    for (unsigned i=0; i<surface.VV.size(); i++)
    {
        glVertex(surface.VV[i]);
        glVertex(surface.VV[i] + surface.VN[i] * len);
    }
    glEnd();

    glPopAttrib();
}

/*******************************************************************************************************

                   .                       .    ..|''||   '||         || '||''''|  ||  '||
  ...   ... ...  .||.  ... ...  ... ...  .||.  .|'    ||   || ...    ...  ||  .   ...   ||    ....
.|  '|.  ||  ||   ||    ||'  ||  ||  ||   ||   ||      ||  ||'  ||    ||  ||''|    ||   ||  .|...||
||   ||  ||  ||   ||    ||    |  ||  ||   ||   '|.     ||  ||    |    ||  ||       ||   ||  ||
 '|..|'  '|..'|.  '|.'  ||...'   '|..'|.  '|.'  ''|...|'   '|...'     || .||.     .||. .||.  '|...'
                        ||                                         .. |'
                       ''''                                         ''
********************************************************************************************************/

void outputObjFile(ostream &out, const Surface &surface)
{

    for (unsigned i=0; i<surface.VV.size(); i++)
        out << "v  "
            << surface.VV[i][0] << " "
            << surface.VV[i][1] << " "
            << surface.VV[i][2] << endl;

    for (unsigned i=0; i<surface.VN.size(); i++)
        out << "vn "
            << surface.VN[i][0] << " "
            << surface.VN[i][1] << " "
            << surface.VN[i][2] << endl;

    out << "vt  0 0 0" << endl;

    for (unsigned i=0; i<surface.VF.size(); i++)
    {
        out << "f  ";
        for (unsigned j=0; j<3; j++)
        {
            unsigned a = surface.VF[i][j]+1;
            out << a << "/" << "1" << "/" << a << " ";
        }
        out << endl;
    }
}
