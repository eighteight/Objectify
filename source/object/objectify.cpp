
#include "c4d.h"
#include "c4d_symbols.h"
#include "c4d_tools.h"
#include "lib_splinehelp.h"
#include "ge_dynamicarray.h"
#include "octobjectify.h"
#include <vector>
#include <string>
#include <iostream>
#include "Triangulator.h"
// unique ID obtained from www.plugincafe.com
#define ID_OBJECTIFY 1031255

class Objectify : public ObjectData
{
private:
    LineObject *PrepareSingleSpline(BaseObject *generator, BaseObject *op, Matrix *ml, HierarchyHelp *hh, Bool *dirty);
    void Transform(PointObject *op, const Matrix &m);
    void DoRecursion(BaseObject *op, BaseObject *child, GeDynamicArray<Vector> &points, Matrix ml);
    Random rng;
    Bool isCalculated;
    GeDynamicArray<GeDynamicArray<Vector> > splineAtPoint;
    LONG prvsFrame = 0;
    Triangulator tri;
public:
    virtual BaseObject* GetVirtualObjects(BaseObject *op, HierarchyHelp *hh);
    virtual Bool Init(GeListNode *node);
    
    static NodeData *Alloc(void) { return gNew Objectify; }
};

void Objectify::Transform(PointObject *op, const Matrix &m)
{
    Vector        *padr=op->GetPointW();
    LONG        pcnt=op->GetPointCount(),i;
    
    for (i=0; i<pcnt; i++)
        padr[i]*=m;
    
    op->Message(MSG_UPDATE);
}

Bool Objectify::Init(GeListNode *node)
{
    BaseObject                *op   = (BaseObject*)node;
    BaseContainer *data = op->GetDataInstance();
    
    data->SetReal(CTTSPOBJECT_MAXSEG,30.);
    data->SetBool(CTTSPOBJECT_REL,TRUE);
    data->SetLong(SPLINEOBJECT_INTERPOLATION,SPLINEOBJECT_INTERPOLATION_ADAPTIVE);
    isCalculated = FALSE;
    GePrint("Objectify by http://twitter.com/eight_io for Cinema 4D r14");
    return TRUE;
}

BaseObject *Objectify::GetVirtualObjects(BaseObject *op, HierarchyHelp *hh)
{
    BaseDocument *doc = op->GetDocument();
    BaseContainer *data = op->GetDataInstance();
    BaseObject* obj = (BaseObject*)data->GetLink(CTT_OBJECT_LINK,doc,Obase);

    if (!obj) return NULL;
    
    if (obj->GetTypeName() != "Spline"){
        return NULL;
    }
    
    SplineObject* spline = (SplineObject*) obj;

    LONG crntFrame = doc->GetTime().GetFrame(doc->GetFps());

    LONG delta = data->GetLong(CTTSPOBJECT_WINDOW,1);
    LONG strtFrame = crntFrame - delta;
    strtFrame = strtFrame<0?0:strtFrame;
    // start new list
    op->NewDependenceList();
    
    // check cache for validity and check master object for changes
    Bool dirty = op->CheckCache(hh) || op->IsDirty(DIRTYFLAGS_DATA);

    LONG trck = 0;

    // if child list has been modified
    if (!dirty) dirty = !op->CompareDependenceList();
    
    // mark child objects as processed
    op->TouchDependenceList();
    
    // if no change has been detected, return original cache
    if (!dirty) return op->GetCache(hh);
    
    
    AutoAlloc<SplineHelp> splineHelp;
    
    vector<vector<float> > points;
    for (LONG i = 0; i < spline->GetSegmentCount(); i++){
        Vector p = spline->GetSplinePoint(0.5, i);
        vector<float> point;
        point.push_back(p.x);
        point.push_back(p.y);
        point.push_back(p.z);
        points.push_back(point);
    }
    
    vector<vector<float> > surfacePoints;
    vector<vector<int> > surfaceIndeces;
    tri.triangulate(points, surfacePoints, surfaceIndeces);
    
    if (surfaceIndeces.size() == 0){
        return NULL;
    }
    
    PolygonObject* myPoly = PolygonObject::Alloc(surfacePoints.size(),surfaceIndeces.size());
    Vector* ppoints = myPoly->GetPointW();
    vector<float> sp;
    
    for (int t = 0; t < surfacePoints.size()-2; t += 3) {
        
        sp = surfacePoints[t];
        ppoints[t+0] = 100.0*Vector(sp[0],sp[1],sp[2]);
        
        sp = surfacePoints[t+1];
        ppoints[t+1] = 100.0*Vector(sp[0],sp[1],sp[2]);
        
        sp = surfacePoints[t+2];
        ppoints[t+2] = 100.0*Vector(sp[0],sp[1],sp[2]);
    }
    
    for (int t = 0; t < surfaceIndeces.size(); t ++) {
        myPoly->GetPolygonW()[t] = CPolygon(surfaceIndeces[t][0],surfaceIndeces[t][1],surfaceIndeces[t][2], surfaceIndeces[t][2]);
    }
    
    return ToPoly(myPoly);
    
    
    Real maxSeg = data->GetReal(CTTSPOBJECT_MAXSEG,30.);
    Bool relativeMaxSeg  = data->GetBool(CTTSPOBJECT_REL,TRUE);

    
    BaseThread    *bt=hh->GetThread();
    BaseObject* main = BaseObject::Alloc(Onull);
    isCalculated = TRUE;
    StatusSetBar(0);
    StatusSetText("Collecting Points");

    Real distMin = MAXREALr;
    Real distMax = 0.;
    
    //splineAtPoint.FreeArray();
    
    
    SplineObject* emptySpline = SplineObject::Alloc(0, SPLINETYPE_LINEAR);
    ModelingCommandData mcd;
    
    mcd.doc = doc;
    
    mcd.op = emptySpline;
    
    if(!SendModelingCommand(MCOMMAND_JOIN, mcd)){
        return NULL;
    }
    

    prvsFrame = crntFrame;
    StatusClear();
    

    
    //////////////////
    
    main->Message(MSG_UPDATE);
    prvsFrame = crntFrame;
    StatusClear();
    //return main;
Error:

    return NULL;
}

// unique ID obtained from www.plugincafe.com



Bool RegisterObjectify(void)
{
	return RegisterObjectPlugin(ID_OBJECTIFY,GeLoadString(IDS_OBJECTIFY),OBJECT_GENERATOR|OBJECT_INPUT|OBJECT_ISSPLINE|OBJECT_CALL_ADDEXECUTION,Objectify::Alloc,"Octobjectify",AutoBitmap("tsp.tif"),0);
}
