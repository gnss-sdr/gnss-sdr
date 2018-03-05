
extern int opt_debug;

#define	ASN_DEBUG(fmt, args...)	do {		\
		if(opt_debug < 2) break;	\
		fprintf(stderr, fmt, ##args);	\
		fprintf(stderr, " (%s:%d)\n",	\
			__FILE__, __LINE__);	\
	} while(0)

