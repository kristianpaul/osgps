int REG_read[256],REG_write[256];
#define inw_p(x) inpwd(x)
#define outw_p(x,y) outpwd(y,x)

extern int inpwd(unsigned short int);
extern void outpwd(unsigned short int, unsigned short int);

