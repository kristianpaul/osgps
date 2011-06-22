#include "CMatrix3.h"


CMatrix::CMatrix(CMatrix & cm)
{
	Create_Mem(cm.Nrows(),cm.Ncols());
	for(int i=0 ;i<Nrows();i++)
		for(int j=0;j<Ncols();j++)
			p->Mat[i][j] = cm.p->Mat[i][j];

}


CMatrix::CMatrix(int r, int c, double init)
{
 	Create_Mem(r,c);
	for(int i=0;i<r;i++)
		for(int j=0;j<c;j++)
			p->Mat[i][j] = init;

}





CMatrix::CMatrix(int mrows, int columns, double* initvalues)
{
  // create the structure:
  p = new matrep;
  p->r= mrows; 
  p->c = columns; 
  // allocate memory for the actual matrix: 
  p->Mat = new double *[mrows]; 
  for (int x = 0; x < mrows; x++) 
  p->Mat[x] = new double[columns];

  int c = 0; 
  for (int i=0; i< mrows; i++)
  { 
    for (int j = 0; j < columns; j++)
      mval(i,j) =  initvalues[c++]; 
  } 
} 


CMatrix::CMatrix(int mrows, double* initvalues)
{ 
  // create the structure:
//  int n = sizeof(*initvalues);
  p = new matrep; 
  p->r= mrows; 
  p->c = 1; 
  // allocate memory for the actual matrix: 
  p->Mat = new double *[mrows]; 
  for (int x = 0; x < mrows; x++) 
  p->Mat[x] = new double[1];
  int c = 0;
  for (int i=0; i< mrows; i++)
  { 
    for (int j = 0; j < 1; j++)
		mval(i,j) =  initvalues[c++];
  } 

}

CMatrix::CMatrix(char * flag, int dimension,double init)
{  

 int i,j;
 
  p = new matrep; 
  p->r = dimension; 
  p->c = dimension; 
  p->Mat = new double *[dimension]; 
  for (int x = 0; x < dimension; x++) 
    p->Mat[x] = new double[dimension]; 


  switch (flag[0])
  {
  case 'I':
	  for (i=0; i< dimension; i++)
	  {
		for (j = 0; j < dimension; j++) 
			mval(i,j) = (i == j ? 1 : 0);
	  }
	  break;
  case 'D':
	  for (i=0; i< dimension; i++)
	  {
		for (j = 0; j < dimension; j++) 
			mval(i,j) = (i == j ? init : 0);
	  }
	  break;
  case 'S':
	  for (i=0; i< dimension; i++)
	  {
		for (j = 0; j < dimension; j++) 
			mval(i,j) = (i == j ? 0 : init);
	  }

	  break;
  }
 
} 


CMatrix::CMatrix(char * flag, int dimension)
{ 
 
  double init=0.0;
  int i,j;
  p = new matrep; 
  p->r = dimension; 
  p->c = dimension; 
  p->Mat = new double *[dimension]; 
  for (int x = 0; x < dimension; x++) 
    p->Mat[x] = new double[dimension]; 



  switch (flag[0])
  {
  case 'I':
	  for (i=0; i< dimension; i++)
	  {
		for (j = 0; j < dimension; j++) 
			mval(i,j) = (i == j ? 1 : 0);
	  }
	  break;
  case 'D':
	  for (i=0; i< dimension; i++)
	  {
		for (j = 0; j < dimension; j++) 
			mval(i,j) = (i == j ? init : 0);
	  }
	  break;
  case 'S':
	  for (i=0; i< dimension; i++)
	  {
		for (j = 0; j < dimension; j++) 
			mval(i,j) = (i == j ? 0 : init);
	  }

	  break;
  }
} 



CMatrix::~CMatrix(void)
{
	Release_Mem();
}


void CMatrix::Create_Mem(int rows, int cols)
{
	p = new matrep;

	p->r = rows;
	p->c = cols;

  // allocate memory for the actual matrix: 
  p->Mat = new double *[rows]; 
  for (int x = 0; x < rows; x++) 
    p->Mat[x] = new double[cols]; 

  for(int i = 0; i < rows; i++)
	  for(int j = 0;j< cols;j++)
		  p->Mat[i][j] = init_val;

}

void CMatrix::Release_Mem(void)
/* free a float matrix allocated by matrix() */
{   
	{
		for(int x = 0; x < p->r ; x++)
			delete p->Mat[x];
		delete p->Mat;
		delete p;
	}

}

 
CMatrix  CMatrix::operator = (const CMatrix  & cm)
{
	int i,j;
	for(int x = 0; x < p->r ; x++)
	delete p->Mat[x];
	delete p->Mat;
	delete p;

	p = new matrep;

	p->r = cm.Nrows();
	p->c = cm.Ncols();

	// allocate memory for the actual matrix:
	p->Mat = new double *[cm.Nrows()];
	for (i = 0; i < Nrows(); i++)
	p->Mat[i] = new double[cm.Ncols()];

	for(i = 0; i < Nrows(); i++)
		for(j = 0;j< Ncols();j++)
			p->Mat[i][j] = init_val;


	if( (Nrows()!=cm.Nrows()) || (Ncols()!=cm.Ncols())) error("Wrong size matrix assig");
	for(i=0;i<Nrows();i++)
		for(j=0;j<Ncols();j++)
			p->Mat[i][j] = cm.p->Mat[i][j];



	return *this;

}

CMatrix  CMatrix::operator = (const double d)
{

	for(int i = 0 ;i < Nrows();i++)
		for(int j=0;j<Ncols();j++)
			p->Mat[i][j]=d;

	return *this;

}



double &  CMatrix::operator()(int m, int n)
{

	if( (m > rows()) || (n > cols()) || ( m < 1)  || ( n < 1)) 
	{  
	   printf("m=%d,rows=%d  n=%d, cols=%d\n",m,rows(),n,cols());
	   error(" element assigment error");                        
	 }
	return p->Mat[m-1][n-1];
}



double &  CMatrix::operator()(int m)
{
	int n=1;

	if( (m > rows()) || (n > cols()) || ( m < 1)  || ( n < 1)) 
	{
	   printf("m=%d,rows=%d  n=%d, cols=%d\n",m,rows(),n,cols());
	   error(" element assigment error");  
	}                      
	return p->Mat[m-1][n-1];//(*this);
}



CMatrix   CMatrix::operator + (const CMatrix & M)
{

	if( (rows() != M.rows() ) || ( cols() != M.cols()) )error("Wrong size for matrix addition");

	int i,j;

	CMatrix result(rows(),cols());

	for(i=0;i<rows();i++)
	{
		for(j=0;j<cols();j++)
		{
			result.p->Mat[i][j] = p->Mat[i][j] + M.p->Mat[i][j];
		}
	}

	return result;
}

CMatrix   CMatrix::operator + (const double rval)
{

	int i,j;

	CMatrix result(rows(),cols());

	for(i=0;i<rows();i++)
	{
		for(j=0;j<cols();j++)
		{
			result.p->Mat[i][j] = p->Mat[i][j]+ rval;
		}
	}

	return result;
}

CMatrix  CMatrix::operator - (const CMatrix   & M)
{
	if( ( rows() != M.rows() ) || ( cols() != M.cols()))error("Wrong size for matric subtr");
	int i,j;
	CMatrix result(rows(),cols());
	 result = (*this);
	for(i=0;i<rows();i++)
	{
		for(j=0;j<cols();j++)
		{
			result.p->Mat[i][j] = p->Mat[i][j] - M.p->Mat[i][j];
		}
	}

	return result;
}


CMatrix  CMatrix::operator - (const double  rval)
{
	int i,j;
	CMatrix result(rows(),cols());
	result = (*this);

	for(i=0;i<rows();i++)
	{
		for(j=0;j<cols();j++)
		{
			result.p->Mat[i][j] = p->Mat[i][j] - rval;
		}
	}

	return result;
}

CMatrix     operator *   (double f, CMatrix  BM)
{
	CMatrix m(BM.Nrows(),BM.Ncols());

	m = BM*f;
	
	return m;
}

CMatrix  CMatrix::operator * (const CMatrix & M)
{
	if( (cols() != M.rows() ) ) error("wrong matrix size for multiplication");
	CMatrix a(rows(),M.cols());

	int i,j,k, c = cols();
	double sum=0;

	for(i=0;i<rows();i++)
	{
		for(j=0;j<M.cols();j++)
		{
			sum=0.0;
			for(k=0;k<c;k++)
			{
				sum = sum + p->Mat[i][k]*M.p->Mat[k][j];
			}
			a.p->Mat[i][j]=sum;
		}
	}

	return a;
}


CMatrix  CMatrix::operator * (double d)
{

	CMatrix a(rows(),cols());

	int i,j;
	for(i=0;i<rows();i++)
	{
		for(j=0;j<cols();j++)
		{
			a.p->Mat[i][j]=p->Mat[i][j]*d;
		}
	}

	return a;
}


CMatrix  CMatrix::operator / (double d)
{

	CMatrix a(rows(),cols());

	int i,j;
	for(i=0;i<rows();i++)
	{
		for(j=0;j<cols();j++)
		{
			a.p->Mat[i][j]=p->Mat[i][j]/d;
		}
	}

	return a;
}


CMatrix  CMatrix::operator / (CMatrix  M)
{

	CMatrix a(rows(),cols()),b(rows(),cols());

	 a = (*this) * M.inverse();
	return a;
}

//************************************************************
double & CMatrix::val(int row, int col)
{
  if (row < 0 || row >= rows() || col < 0 || col >= cols())
	 error("index out of range");
  return (mval(row,col));
}

CMatrix CMatrix::transpose()
{
//  if(rows() != cols())
//  error("matrix must be square to transpose!\n");
  CMatrix trans(cols(),rows());
  for (int row = 0; row < rows(); row++)
  {
	 for(int col = 0; col < cols(); col++)
	{
		trans.mval(col,row) = mval(row,col);
	}
  }
  return trans;
}

double CMatrix::determinant()
{
  if(rows() != cols()) error("matrix must be square for determinant()");
  CMatrix indx(cols()); // create the "index vector"
  CMatrix B(cols()); // see pp 38. in Numerical Recipes
  int d;
  // perform the decomposition once:
  CMatrix decomp = lu_decompose(indx,d);
  double determinant = d;
  for(int i=0; i < cols() ; i++)
	 determinant *= decomp.mval(i,i);
  return determinant;
}

CMatrix CMatrix::inverse()
{
  if(rows() != cols()) error("matrix must be square for inverse()"); 
  CMatrix Y("I",rows()); // create an identity matrix  
  CMatrix indx(cols()); // create the "index vector"  
  CMatrix B(cols()); // see Press & Flannery 
  int d; 
  // perform the decomposition once: 
  CMatrix decomp = lu_decompose(indx,d); 
  for(int col = 0; col < cols(); col++){ 
    B.copy_column(Y,col,0); 
    decomp.lu_back_subst(indx,B); 
    Y.copy_column(B,0,col); 
  } 
  return Y.transpose(); 
} 

/************************************************************ 
The private support functions for determinant & inverse. 
************************************************************/ 
 
// copy the from_col of mm to the to_col of "this" 
void CMatrix::copy_column(CMatrix& mm, int from_col, int to_col){
  if(rows() != mm.rows())  error("number of rows must be equal for copy_column()"); 
  for(int row=0; row < rows(); row++) 
    mval(row,to_col) = mm.mval(row,from_col); 
} 
 
void CMatrix::switch_columns(int col1, int col2)
{
  int row;
  CMatrix temp(rows()); 
  for(row = 0; row < rows(); row++) 
    // temporarily store col 1: 
    temp.mval(row,0) = mval(row,col1);  
  for(row = 0; row < rows(); row++) 
    mval(row,col1) = mval(row,col2); // move col2 to col1  
  for(row = 0; row < rows(); row++) 
    mval(row,col2) = temp.mval(row,0); // move temp to col2  
} 
 
// make an image of a matrix (used in L-U decomposition) 
void CMatrix::deepcopy(CMatrix& from, CMatrix& to)
{ 
  if(from.rows() != to.rows() || from.cols() != to.cols()) 
    error("matrices must be equal dimensions for deepcopy()"); 
  for(int row = 0; row < from.rows(); row++) {
    for(int col = 0; col < from.cols(); col++) 
      to.mval(row,col) = from.mval(row,col); 
  }
} 
 
// scale a matrix (used in L-U decomposition)  
CMatrix CMatrix::scale()
 { 
  double temp; 
  if(rows() <= 0 || cols() <= 0) error("bad matrix size for scale()"); 
  if(rows() != cols())     error("matrix must be square for scale()"); 
  CMatrix scale_vector(rows()); 
  for (int col = 0; col < cols(); col++){ 
    double maximum = 0; 
    for(int row = 0; row < rows(); row++) 
      if ((temp = (double)fabs(mval(row,col))) > maximum) 
      maximum = temp;  // find max column magnitude in this row
    if(maximum == 0) error("singular matrix in scale()"); 
    scale_vector.mval(col,0) = 1/maximum; // save the scaling  
  } 
  return scale_vector; 
} 
 
CMatrix CMatrix::lu_decompose(CMatrix& indx, int& d )
 { 
/* 
 Returns the L-U decomposition of a matrix. indx is an output 
 vector which records the row permutation effected by the  
 partial pivoting, d is output as +-1 depending on whether the 
 number of row interchanges was even or odd, respectively.   
 This routine is used in combination with lu_back_subst to  
 solve linear equations or invert a matrix. 
*/ 
  if(rows() != cols()) error("Matrix must be square to L-U decompose!\n"); 
  d = 1; // parity check  
  int row,col,k,col_max; // counters  
  double dum; // from the book -- I don't know significance  
  double sum; 
  double maximum; 
  CMatrix lu_decomp(rows(),cols()); 
  // make a direct copy of the original matrix: 
  deepcopy(*this,lu_decomp); 
  CMatrix scale_vector = lu_decomp.scale(); // scale the matrix  
  // The loop over columns of Crout's method:  
  for(row = 0; row < rows(); row++){  
    if (row > 0) { 
      // eqn 2.3.12 except for row=col:  
      for (col = 0; col <= row-1; col++) {  
      sum = lu_decomp.mval(row,col); 
      if(col > 0) { 
        for(k = 0; k <= col-1; k++) 
          sum -= lu_decomp.mval(row,k)*lu_decomp.mval(k,col); 
        lu_decomp.mval(row,col) = sum; 
      } 
      } 
    } 
    // Initialize for the search for the largest pivot element: 
    maximum = 0;  
    // i=j of eq 2.3.12 & i=j+1..N of 2.3.13: 
    for(col=row; col <= cols()-1; col++){  
      sum = lu_decomp.mval(row,col); 
      if(row > 0){ 
      for(k=0; k <= row-1; k++) 
        sum -=  lu_decomp.mval(k,col) * lu_decomp.mval(row,k); 
      lu_decomp.mval(row,col) = sum; 
      } 
      // figure of merit for pivot: 
      dum = scale_vector.mval(col,0) * fabs(sum); 
      if (dum >= maximum){ // is it better than the best so far?
      col_max = col; 
      maximum = dum; 
      } 
    } 
    // Do we need to interchange rows?  
    if(row != col_max) { 
		lu_decomp.switch_columns(col_max,row); // Yes, do so...
      d *= -1;  // ... and change the parity of d  
      // also interchange the scale factor: 
      dum = scale_vector.mval(col_max,0);  
      scale_vector.mval(col_max,0) = scale_vector.mval(row,0); 
      scale_vector.mval(row,0) = dum;  
    } 
    indx.mval(row,0) = col_max; 
    // Now, finally, divide by the pivot element: 
    if(row != rows() -1){   
      if(lu_decomp.mval(row,row) == 0)  
		  lu_decomp.mval(row,row) = 1e-20;
      // If the pivot element is zero the matrix is  
      // singular (at least to the precision of the  
      // algorithm).  For some applications on singular  
      // matrices, it is desirable to substitute tiny for zero 
      dum = 1/lu_decomp.mval(row,row); 
		for(col=row+1; col <= cols()-1; col++)
      lu_decomp.mval(row,col) *= dum; 
    } 
  } 
  if(lu_decomp.mval(rows()-1,cols()-1) == 0)  
	 lu_decomp.mval(rows()-1,cols()-1) = 1e-20;
  return lu_decomp; 
} 
 
void CMatrix::lu_back_subst(CMatrix& indx, CMatrix& b)
{ 
/*  
 Solves the set of N linear equations A*X = B.  Here "this"  
 is the LU-decomposition of the matrix A, determined by the 
 routine lu_decompose(). Indx is input as the permutation  
 vector returned  by lu_decompose().  B is input as the  
 right-hand side vector B,  and returns with the solution  
 vector X.  This routine takes into  account the possibility  
 that B will begin with many zero elements,  so it is efficient 
 for use in matrix inversion.   See pp 36-37 in  
 Press & Flannery. 
*/  
  if(rows() != cols())  
    error ("non-square lu_decomp matrix in lu_back_subst()"); 
  if(rows() != b.rows()) 
    error("wrong size B vector passed to lu_back_subst()"); 
  if(rows() != indx.rows()) 
    error("wrong size indx vector passed to lu_back_subst()"); 
  int row,col,ll; 
  int ii = 0; 
  double sum; 
  for(col=0;col < cols(); col++){ 
    ll= (int)indx.mval(col,0); 
    sum = b.mval(ll,0); 
    b.mval(ll,0) = b.mval(col,0); 
    if (ii >= 0) 
      for(row = ii; row <= col-1; row++) 
      sum -= mval(row,col) * b.mval(row,0); 
    else if(sum != 0) 
      ii = col; 
    b.mval(col,0) = sum; 
  } 
  for(col = cols() -1; col >= 0; col--){ 
    sum = b.mval(col,0); 
    if (col < cols() -1) 
      for (row = col + 1; row <= rows()-1; row++) 
      sum -= mval(row,col) * b.mval(row,0); 
    // store a component of the soln vector X: 
    b.mval(col,0) = sum/mval(col,col);  
  } 
}

void DebugOut(Matrix v)
{
	int n = v.Nrows();
	int m = v.Ncols();
	int i,j;

	for (i=1;i<=n;i++)
	{
	  for (j=1;j<=m;j++)
	  {
			printf("%10.6f ",v(i,j));
	  }
	  printf("\n");
	}

}

