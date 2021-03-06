CONTAINER Octobjectify
{
	NAME Octobjectify;
	INCLUDE Obase;

	GROUP ID_OBJECTPROPERTIES
	{
		LINK CTT_OBJECT_LINK { ACCEPT { Ospline; } }
        REAL POSITION_ALONG_SPLINE {MIN 0.0; MAX 10000.0; CUSTOMGUI REALSLIDER; STEP 0.0001; MINSLIDER 0.0; MAXSLIDER 10000.0;}
		LONG KSEARCH_NEIGHBORS { MIN 3; MAX 1000; CUSTOMGUI LONGSLIDER; }
        REAL KSEARCH_MU {MIN 0.1; CUSTOMGUI REALSLIDER; STEP 0.1; MINSLIDER 0.1; MAXSLIDER 100.0;}
        LONG TRIANGULATION_MAX_NEIGHBORS { MIN 3; MAX 1000; CUSTOMGUI LONGSLIDER; }
        REAL TRIANGULATION_MAX_SEARCH_RADIUS { UNIT METER;	MIN 0.1; CUSTOMGUI REALSLIDER; STEP 0.1; MINSLIDER 0.0; MAXSLIDER 1000.0;}
        BOOL USE_MLS {}
	}
}
