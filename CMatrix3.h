#ifndef __CMATRIX__
#define __CMATRIX__

#include <stdio.h>
#include <stdlib.h>
#include <iostream.h>
#include <fstream.h>
#include <iomanip.h>
#include <string.h>
#include <math.h>
#include <malloc.h>

void test_mat ();               // Function to test matrix class

typedef class CMatrix Matrix;
typedef class CMatrix ColumnVector;
typedef class CMatrix DiagonalMatrix;
typedef class CMatrix SymmetricMatrix;



struct matrep
{
  double **Mat;
  int r;
  int c;
};

typedef double p_D;

//*****************************************************************************
//  Alberto N. Perez
//
//      CMatrix class
//                              A basic matrix class that can be embedded into a real time
//                              system such as navigation or flight control.
//                              All the source is available with in one file, and it can
//                              be review for a flight system verification.
//                              This class will be use as basic building block for a vector
//                              and a quaternion class 
//                              
//                              members
//                                      matrix and scalar addition
//                                      matrix and scalar subtraction   
//                                      matrix and scalar multiplication
//                                      inverse
//                                      determinant
//                                      max and min element
//                                      variance and mean of elements
//  Notes
//                              The  matrix storage and handling and inverse matrix member are mainly
//                              derived from reference #1.
//                              Reference (2) was used to verify reference (1).
//                              
//                              
//
//      References
//                              (1) "C++ Inside & Out", by Bruce Eckel, Chapter #9
//                              (2) "Numerical Recepies" in C, Second Edition, Chapter #2
//                              
//
//*****************************************************************************
typedef double Real;

class CMatrix
{
private:
  matrep * p;
  double init_val;
public:
    CMatrix (int r = 1, int c = 1, double init = 0);
    CMatrix (int r, int c, double *initval);
    CMatrix (char *flag, int dimension, double init);
    CMatrix (char *flag, int dimension);
    CMatrix (char *initfile);
    CMatrix::CMatrix (int mrows, double *initvalues);

   ~CMatrix (void);

//************************************************
//      void ReSize(int r,int c,double init);
//      void ReSize(int r,int c);
//      void ReSize(int r);

  void CMatrix::Create_Mem (int r, int c);
  void Release_Mem (void);
  void Release (void)
  {
    Release_Mem ();
  };

  CMatrix (CMatrix & cm);       // Copy constructor     

  CMatrix operator + (const CMatrix & M);       // Matrix addition
  CMatrix operator + (const double rval);       // Scalar addition 
  CMatrix operator - (const CMatrix & M);       // Matrix subtraction
  CMatrix operator - (const double rval);       // Scalar subtraction
  CMatrix operator * (const CMatrix & M);       // Matrix multiplication
  CMatrix operator * (double d);        // Scalar multiplication
  CMatrix operator / (CMatrix M);       // Matrix Division
  CMatrix operator / (double d);        // Scalar Division

  // Assign a value to an array element    //
  CMatrix operator = (const CMatrix & M);
  CMatrix operator = (const double d);
  double &operator () (int m, int n);
  double &operator () (int m);

//      operator double() const;
  //***************************************//
  // Read the values from an array element //
  //     operator double();
  //***************************************//
  int rows () const
  {
    return p->r;
  };                            // rows in matrix
  int cols () const
  {
    return p->c;
  };                            // cols in matrix  
  int Nrows () const
  {
    return p->r;
  };                            // rows in matrix
  int Ncols () const
  {
    return p->c;
  };                            // cols in matrix  
  //****************************************************
  CMatrix transpose ();         // transpose a square matrix 
  CMatrix t ()
  {
    return transpose ();
  };                            // Transpose shell for newmat
  double determinant ();        // Determinant
  CMatrix inverse ();           // Inverse
  CMatrix i ()
  {
    return inverse ();
  }                             // Inverse shell for newmat

  //****************************************************
private:
  double &val (int row, int col);
  double &mval (int row, int col)
  {
    return (p->Mat[row][col]);
  };
  double &mval (int row)
  {
    return (p->Mat[row][0]);
  };
  CMatrix scale (void);
  void copy_column (CMatrix & mm, int from_col, int to_col);
  void deepcopy (CMatrix & from, CMatrix & to);
  CMatrix lu_decompose (CMatrix & indx, int &d);
  void switch_columns (int col1, int col2);
  void lu_back_subst (CMatrix & indx, CMatrix & b);

public:

  void display (void);
  void error (char *c)
  {
    printf ("%s", c);
    getchar ();
    exit (1);
  };
  void error (char *msg1, char *msg2)
  {
    cerr << "matrix error: " << msg1 << " " << msg2 << endl;
    exit (1);
  }
  void write_standard (char *filename, char *msg);

};


#endif
