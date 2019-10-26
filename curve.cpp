#include "curve.h"
#include "extra.h"
#ifdef WIN32
#include <windows.h>
#endif
#include <GL/gl.h>

// Additional includes:
#include<cmath> // for pow()
#include <string> // for using strings

using namespace std;

namespace
{
    // Approximately equal to.  We don't want to use == because of
    // precision issues with floating point.
    inline bool approx( const Vector3f& lhs, const Vector3f& rhs )
    {
        const float eps = 1e-8f;
        return ( lhs - rhs ).absSquared() < eps;
    }
}

Vector3f x (Vector3f a, Vector3f b)
{
  Vector3f axb = Vector3f (
                            a[1] * b[2] - a[2] * b[1],
                            a[2] * b[0] - a[0] * b[2],
                            a[0] * b[1] - a[1] * b[0]
                          );
  return axb;
}


//
//                          '||  '||''|.                    ||
//   ....  .... ...  ....    ||   ||   ||    ....  ......  ...    ....  ... ..
// .|...||  '|.  |  '' .||   ||   ||'''|.  .|...|| '  .|'   ||  .|...||  ||' ''
// ||        '|.|   .|' ||   ||   ||    || ||       .|'     ||  ||       ||
//  '|...'    '|    '|..'|' .||. .||...|'   '|...' ||....| .||.  '|...' .||.
//
//
Curve evalBezier( const vector< Vector3f >& P, unsigned steps)
{
    vector< CurvePoint > BezierCurve;
    Matrix4f ControlPolygon;
    // Check
    cerr << " P SIZE: " << P.size() << endl;
    if( P.size() < 4 || P.size() % 3 != 1 )
    {
        cerr << "evalBezier must be called with 3n+1 control points." << endl;
        exit( 0 );
    }
    // TODO:
    // You should implement this function so that it returns a Curve
    // (e.g., a vector< CurvePoint >).  The variable "steps" tells you
    // the number of points to generate on each piece of the spline.
    // At least, that's how the sample solution is implemented and how
    // the SWP files are written.  But you are free to interpret this
    // variable however you want, so long as you can control the
    // "resolution" of the discretized spline curve with it.

    // Make sure that this function computes all the appropriate
    // Vector3fs for each CurvePoint: V,T,N,B.
    // [NBT] should be unit and orthogonal.

    // Also note that you may assume that all Bezier curves that you
    // receive have G1 continuity.  Otherwise, the TNB will not be
    // be defined at points where this does not hold.

    cerr << "\t>>> evalBezier has been called with the following input:" << endl;

    cerr << "\t>>> Bezier: Control points (type vector< Vector3f >): "<< endl;
    ControlPolygon = Matrix4f(0);
    for( unsigned i = 0; i < P.size(); ++i )
    {
        cerr << "\t\t>>> " << "x=" << P[i].x()
                         << ", y=" << P[i].y()
                         << ", z=" << P[i].z() << endl;
        Vector4f col = Vector4f(P[i], 0);
        ControlPolygon.setCol(i, col);
    }

    cerr << "\t>>> Steps (type steps): " << steps << endl;

    cerr << " ... populating the curve ... " << endl;

    cerr << "\t>>> Control Polygon: " << endl;
    ControlPolygon.print();

    cerr << "\t>>> Control Polygon x Bernstein Basis: " << endl;

    Matrix4f BernsteinBasis = Matrix4f
    (
      1, -3,  3, -1,
      0,  3, -6,  3,
      0,  0,  3, -3,
      0,  0,  0,  1
    );

    Matrix4f GB = ControlPolygon * BernsteinBasis;
    GB.print();

    cerr << "\t>>> Curve " << endl;

    for ( double i = 0.0; i <= 1.0; i = i + 1.0/steps )
    {
      Vector4f MonomialBasis = Vector4f(1, i, pow(i,2), pow(i,3));
      Vector4f dydx_MonomialBasis = Vector4f(0, 1, 2*i, 3*pow(i,2));
      Vector4f d2ydx2_MonomialBasis = Vector4f(0, 0, 2, 6*i);
      CurvePoint point; // current iteration points of the curve
      CurvePoint B_1point; // previous iteration Binormal
      Vector4f GBT = GB * MonomialBasis;

      // convertion to Vector3f, so we can write it to 'point.V':
      Vector3f GBT3f = Vector3f ( GBT.x(), GBT.y(), GBT.z() );
      point.V = GBT3f;

      Vector4f dydx_GBT = GB * dydx_MonomialBasis;
      point.T = Vector3f(dydx_GBT.x(), dydx_GBT.y(), dydx_GBT.z()).normalized();

      if (i == 0.0)
      {
        // Initialize the binormal ...
        // (set it to just face up for the first iteration):
        Vector3f findingOrthoT_B = Vector3f( 0, 0, 1 );

        point.B = x(point.T, findingOrthoT_B);
      }

      // Normal vector is second derivative
      Vector4f d2ydx2_GBT = GB * d2ydx2_MonomialBasis;

      if (i > 0.0)
      {
        B_1point = BezierCurve.back();

        point.N = x(B_1point.B, point.T).normalized();
        point.B = x(point.T, point.N);
      }

      else
      {
        Vector3f N3 = Vector3f (
                                  point.B[1] * point.T[2] - point.B[2] * point.T[1],
                                  point.B[2] * point.T[0] - point.B[0] * point.T[2],
                                  point.B[0] * point.T[1] - point.B[1] * point.T[0]
                                );
        point.N = x(point.B, point.T);
      }

      BezierCurve.push_back(point);

    }

    cerr << "\t>>> Bezier: Returning populated curve:" << endl;

    // Right now this will just return this empty curve.
    return BezierCurve;

    // drawCurve( curve, 0 )
}


//
//                          '||  '||''|.                   '||   ||
//   ....  .... ...  ....    ||   ||   ||   ....  ... ...   ||  ...  .. ...     ....
// .|...||  '|.  |  '' .||   ||   ||'''|.  ||. '   ||'  ||  ||   ||   ||  ||  .|...||
// ||        '|.|   .|' ||   ||   ||    || . '|..  ||    |  ||   ||   ||  ||  ||
//  '|...'    '|    '|..'|' .||. .||...|'  |'..|'  ||...'  .||. .||. .||. ||.  '|...'
//                                                 ||
//                                                ''''
Curve evalBspline( const vector< Vector3f >& P, unsigned steps )
{
    vector< CurvePoint > BSplineCurve;
    Matrix4f ControlPolygon;
    // Check
    if( P.size() < 4 )
    {
        cerr << "evalBspline must be called with 4 or more control points." << endl;
        exit( 0 );
    }

    // TODO:
    // It is suggested that you implement this function by changing
    // basis from B-spline to Bezier.  That way, you can just call
    // your evalBezier function.

    cerr << "\t>>> evalBSpline has been called with the following input:" << endl;

    cerr << "\t>>> BSpline: Control points (type vector< Vector3f >): "<< endl;
    cerr << "Found: " << P.size() << " points." << endl;
    for( unsigned offset = 0; offset < P.size(); offset++ )
    {
      cerr << "offset: " << offset << endl;
      int untilcolumn;
      if (P.size() - offset < 4) untilcolumn = P.size() - offset;
      else untilcolumn = 4;
      int columnID = 0;
      for( unsigned i = offset; i < offset+untilcolumn; ++i )
      {
          if (i == P.size()) break;
          cerr << "\t\t>>> " << i << ") x=" << P[i].x()
                           << ", y=" << P[i].y()
                           << ", z=" << P[i].z() << endl;
          Vector4f col = Vector4f(P[i], 0);
          ControlPolygon.setCol(columnID, col);
          columnID++;
      }

      cerr << "\t>>> Steps (type steps): " << steps << endl;

      cerr << "\t>>> Control Polygon x B-Spline Basis: " << endl;
      Matrix4f BSplineBasis = Matrix4f
      (
        1./6., -3./6.,  3./6., -1./6.,
        4./6.,  0./6., -6./6.,  3./6.,
        1./6.,  3./6.,  3./6., -3./6.,
        0./6.,  0./6.,  0./6.,  1./6.
      );

      Matrix4f GB = ControlPolygon * BSplineBasis;
      /* Changing basis (if regular form is GBT, ...
         ... right now making it GB-1BT and 'evalBezier' ...
         ... will convert it back to BGT [B-1 is the inverse] ) */

      Matrix4f BernsteinBasis = Matrix4f
      (
        1, -3,  3, -1,
        0,  3, -6,  3,
        0,  0,  3, -3,
        0,  0,  0,  1
      );
      // updating the Geometry x Basis (GB-1B)
      GB = GB * BernsteinBasis.inverse();;

      vector< Vector3f > Points;

      Vector3f point1 = Vector3f( GB.getCol(0)[0], GB.getCol(0)[1], GB.getCol(0)[2] );
      Points.push_back(point1);
      Vector3f point2 = Vector3f( GB.getCol(1)[0], GB.getCol(1)[1], GB.getCol(1)[2] );
      Points.push_back(point2);
      Vector3f point3 = Vector3f( GB.getCol(2)[0], GB.getCol(2)[1], GB.getCol(2)[2] );
      Points.push_back(point3);
      Vector3f point4 = Vector3f( GB.getCol(3)[0], GB.getCol(3)[1], GB.getCol(3)[2] );
      Points.push_back(point4);

      cerr << "Size of the current BSplineCurve: " << BSplineCurve.size() << endl;
      cerr << " ---------> SENDING POINTS TO BEZIER <---------- " << endl;
      vector< CurvePoint > newCurveToAdd = evalBezier( Points, steps );

      cerr << "Adding " << newCurveToAdd.size() << " more points" << endl;

      for( unsigned i = 0; i < newCurveToAdd.size(); i++ )
      {
        // cerr << i << ") ";
        // newCurveToAdd[i].V.print();
        // newCurveToAdd[i].N.print();
        // newCurveToAdd[i].B.print();
        // newCurveToAdd[i].T.print();


        CurvePoint testpoint;

        for ( float singleRotation = (2*3.14)/20; singleRotation <= 2*3.14; singleRotation += (2*3.14)/20 )
        {
          Matrix4f matrixOfOnes(1);
          // translate the 3x1 profile vector by the matrix of ones:
          // (just so we are able to apply transforms later by multiplying it with a 4x4 matrix)
          Matrix4f profileMatrix = matrixOfOnes.translation(newCurveToAdd[i].V);

          // this gets the necessary rotation matrix (on Y axis)
          Matrix4f rotationMatrix = matrixOfOnes.rotateY(singleRotation);
          // finally, this multiplication rotates the vector (represented by a matrix)
          Matrix4f rotatedYmatrix = rotationMatrix * profileMatrix;

          // get the last column (where the homogenious vector is)
          Vector4f rotatedHomogeniousVector = rotatedYmatrix.getCol(3);

          testpoint.V = rotatedHomogeniousVector.xyz();
          BSplineCurve.push_back(testpoint);
        }

        BSplineCurve.push_back(newCurveToAdd[i]);
      }
    }


    /* ALTERNATIVE WAY OF COMPUTATION, ...
       ... without calling the 'evalBezier' function ...
       nor computing the inverse and doing some tranformations: */

    /*
    for ( double i = 0.0; i <= 1.0; i = i + 1.0/steps )
    {
      cerr << i << endl;
      Vector4f MonomialBasis = Vector4f(1, i, pow(i,2), pow(i,3));

      CurvePoint point;
      Vector4f GBT = GB * MonomialBasis;

      // convertion to Vector3f, so we can write it to 'point.V':
      Vector3f GBT3f = Vector3f ( GBT.x(), GBT.y(), GBT.z() );
      point.V = GBT3f;

      // appending (adding) to the total point vector (list)
      BSplineCurve.push_back(point);

    }
    */

    cerr << "\t>>> 258Returning populated B-Spline curve." << endl;
    cerr << "Size of the finished BSplineCurve: " << BSplineCurve.size() << endl;
    // Return a BSpline Curve (Vector of CurvePoint points)

    // for( unsigned i = 0; i < BSplineCurve.size(); i++ )
    // {
    //   cerr << "->" << i << ") ";
    //   cerr << "\tV\t";
    //   BSplineCurve[i].V.print();
    //   cerr << "\tN\t";
    //   BSplineCurve[i].N.print();
    //   cerr << "\tB\t";
    //   BSplineCurve[i].B.print();
    //   cerr << "\tT\t";
    //   BSplineCurve[i].T.print();
    //   cerr << endl;
    // }





    return BSplineCurve;
}



//                          '||    ..|'''.|  ||                  '||
//   ....  .... ...  ....    ||  .|'     '  ...  ... ..    ....   ||    ....
// .|...||  '|.  |  '' .||   ||  ||          ||   ||' '' .|   ''  ||  .|...||
// ||        '|.|   .|' ||   ||  '|.      .  ||   ||     ||       ||  ||
//  '|...'    '|    '|..'|' .||.  ''|....'  .||. .||.     '|...' .||.  '|...'
//
Curve evalCircle( float radius, unsigned steps )
{
    // This is a sample function on how to properly initialize a Curve
    // (which is a vector< CurvePoint >).

    // Preallocate a curve with steps+1 CurvePoints
    Curve R( steps+1 );

    // Fill it in counterclockwise
    for( unsigned i = 0; i <= steps; ++i )
    {
        // step from 0 to 2pi
        float t = 2.0f * M_PI * float( i ) / steps;

        // Initialize position
        // We're pivoting counterclockwise around the y-axis
        R[i].V = radius * Vector3f( cos(t), sin(t), 0 );

        // Tangent vector is first derivative
        R[i].T = Vector3f( -sin(t), cos(t), 0 );

        // Normal vector is second derivative
        R[i].N = Vector3f( -cos(t), -sin(t), 0 );

        // Finally, binormal is facing up.
        R[i].B = Vector3f( 0, 0, 1 );
    }

    return R;
}


 //     '||                                ..|'''.|
 //   .. ||  ... ..   ....   ... ... ... .|'     '  ... ...  ... ..  .... ...   ....
 // .'  '||   ||' '' '' .||   ||  ||  |  ||          ||  ||   ||' ''  '|.  |  .|...||
 // |.   ||   ||     .|' ||    ||| |||   '|.      .  ||  ||   ||       '|.|   ||
 // '|..'||. .||.    '|..'|'    |   |     ''|....'   '|..'|. .||.       '|     '|...'
 //
void drawCurve( const Curve& curve, float framesize )
{
    // Save current state of OpenGL
    glPushAttrib( GL_ALL_ATTRIB_BITS );

    // Setup for line drawing
    glDisable( GL_LIGHTING );
    glColor4f( 1, 1, 1, 1 );
    glLineWidth( 1 );

    // Draw curve
    glBegin( GL_LINE_STRIP );
    for( unsigned i = 0; i < curve.size(); ++i )
    {
        glVertex( curve[ i ].V );
    }
    glEnd();

    glLineWidth( 1 );

    // Draw coordinate frames if framesize nonzero
    if( framesize != 0.0f )
    {
        Matrix4f M;

        for( unsigned i = 0; i < curve.size(); ++i )
        {
            M.setCol( 0, Vector4f( curve[i].N, 0 ) );
            M.setCol( 1, Vector4f( curve[i].B, 0 ) );
            M.setCol( 2, Vector4f( curve[i].T, 0 ) );
            M.setCol( 3, Vector4f( curve[i].V, 1 ) );

            glPushMatrix();
            glMultMatrixf( M );
            glScaled( framesize, framesize, framesize );
            glBegin( GL_LINES );
            // normals (red):
            // glColor3f(red, green, blue)
            // glVertex3d( 0, 0, 0 )
            // glVertex3d( 1, 0, 0 ) which vertex to show (if 0,1,1 will draw B+T, 0,0,1 - T)
            glColor3f( 1, 0, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 1, 0, 0 );
            // binormals B (green)
            glColor3f( 0, 1, 0 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 1, 0 );
            // recolored T's to cyan for better normal contrast: (0,1,1) instead of (0,0,1)
            // glColor3f(red, green, blue):
            glColor3f( 0, 1, 1 ); glVertex3d( 0, 0, 0 ); glVertex3d( 0, 0, 1 );
            glEnd();
            glPopMatrix();
        }
    }

    // Pop state
    glPopAttrib();
}
