#ifndef NULL
#define NULL 0
#endif
#define DELETED -2

extern int triangulate, sorted, plot, debug;

struct	Freenode	{
struct	Freenode	*nextfree;
};
struct	Freelist	{
struct	Freenode	*head;
int			nodesize;
};
char *getfree();
char *malloc();
char *myalloc();

extern float xmin, xmax, ymin, ymax, deltax, deltay;


struct Point	{
float x,y;
};

/* structure used both for sites and for vertices */
struct Site	{
struct	Point	coord;
int		sitenbr;
int		refcnt;
};


extern struct	Site	*sites;
extern int		nsites;
extern int		siteidx;
extern int		sqrt_nsites;
extern int		nvertices;
extern struct 	Freelist sfl;
extern struct	Site	*bottomsite;


struct Edge	{
double		a,b,c;
struct	Site 	*ep[2];
struct	Site	*reg[2];
int		edgenbr;
};
#define le 0
#define re 1
extern int nedges;
extern struct	Freelist efl;

int has_endpoint(),right_of();
struct Site *intersect();
float dist();
struct Point PQ_min();
struct Halfedge *PQextractmin();
struct Edge *bisect();

struct Halfedge {
struct Halfedge	*ELleft, *ELright;
struct Edge	*ELedge;
int		ELrefcnt;
char		ELpm;
struct	Site	*vertex;
double		ystar;
struct	Halfedge *PQnext;
};

extern struct   Freelist	hfl;
extern struct	Halfedge *ELleftend, *ELrightend;
extern int 	ELhashsize;
extern struct	Halfedge **ELhash;
struct	Halfedge *HEcreate(), *ELleft(), *ELright(), *ELleftbnd();
struct	Site *leftreg(), *rightreg();


extern int PQhashsize;
extern struct	Halfedge *PQhash;
extern struct	Halfedge *PQfind();
extern int PQcount;
extern int PQmin;
int PQempty();


