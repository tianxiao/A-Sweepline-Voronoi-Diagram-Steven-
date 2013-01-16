#include <iostream.h>
#include "SosUtil.h"

int main()
{
	char str1[] = "this is the main string";
	char str2[] = "Is the";

	char str3[] = "it should fail for this string";
	char str4[] = "blah";

	cout<<"\nstr2 IN str1 = "<<SosUtil::firstInstance(str1,str2, true);
	cout<<"\nstr4 IN str3 = "<<SosUtil::firstInstance(str3,str4);
	
	cout<<endl;
	return 0;
}

