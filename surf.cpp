#include "surf.h"
#include "extra.h"
using namespace std;

namespace
{

    // We're only implenting swept surfaces where the profile curve is
    // flat on the xy-plane.  This is a check function.
    static bool checkFlat(const Curve &profile)
    {
        // cerr << "profile size: " << profile.size() << endl;
        for (unsigned i=0; i<profile.size(); i++)
        { // if profile is empty, just returns true (why it works without any normals etc ...
          // cerr << "V: "; profile[i].V.print();
          // cerr << "T: "; profile[i].T.print();
          // cerr << "N: "; profile[i].N.print();

          cerr << i << ") V:" << profile[i].V[2] << " T:" << profile[i].T[2] << " N:" << profile[i].N[2] << endl;
          if (profile[i].V[2] != 0.0 ||
              profile[i].T[2] != 0.0 ||
              profile[i].N[2] != 0.0)
              {
                cerr << i << ") V:" << profile[i].V[2] << " T:" << profile[i].T[2] << " N:" << profile[i].N[2] << endl;
                cerr << "V: "; profile[i].V.print();
                cerr << "T: "; profile[i].T.print();
                cerr << "N: "; profile[i].N.print();
                cerr << "returning false" << endl;
                // return false;
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

    std::vector< std::vector<Vector3f> > surfacePoints;

    for( unsigned i = 0; i < profile.size(); i++ )
    {
      int profile_number = i;
      // int surface_point_index = 0;
      // cerr << "steps:" << steps << endl;
      int rotation_number = 0;
      std::vector <Vector3f> rotationsOfOneProfile;
      for ( float singleRotation = (2*3.14)/steps; singleRotation <= 2*3.14; singleRotation += (2*3.14)/steps )
      {
        surface.VV.push_back( rotateVertexAroundAxis_y( profile[i], singleRotation ) );
        surface.VN.push_back( rotateNormalAroundAxis_y( profile[i], singleRotation ) );

        rotationsOfOneProfile.push_back(rotateVertexAroundAxis_y( profile[i], singleRotation ));

        // Tup3u triangle(1u,2u,3u);
        //
        // surface.VF.push_back(triangle);

      }
      surfacePoints.push_back(rotationsOfOneProfile);

    }

    for( unsigned currentProfile = 0; currentProfile < profile.size(); currentProfile++ )
    {
      unsigned rotation_number = 0;
      for ( float singleRotation = (2*3.14)/steps; singleRotation <= 2*3.14; singleRotation += (2*3.14)/steps )
      {
          // Tup3u triangle
          // (
          //   (currentProfile * steps) + rotation_number,
          //   (currentProfile * steps) + rotation_number + 1,
          //   ((currentProfile + 1) * steps) + rotation_number + 1
          // );

          // Tup3u triangle
          // (
          //   ((currentProfile + 1) * steps) + rotation_number + 1,
          //   (currentProfile * steps) + rotation_number,
          //   (currentProfile * steps) + rotation_number + 1
          // );

          Tup3u triangle
          (
            ((currentProfile + 1) * steps) + rotation_number + 1,
            (currentProfile * steps) + rotation_number + 1,
            (currentProfile * steps) + rotation_number
          );

          // cerr << "----------- TRIANGLE -------------" << endl;
          // cerr << triangle[0] << endl;
          // cerr << triangle[1] << endl;
          // cerr << triangle[2] << endl << endl;

          surface.VF.push_back(triangle);

          Tup3u bottomtriangle
          (
            ((currentProfile + 1) * steps) + rotation_number,
            ((currentProfile + 1) * steps) + rotation_number + 1,
            (currentProfile * steps) + rotation_number
          );

          surface.VF.push_back(bottomtriangle);

          rotation_number++;
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
