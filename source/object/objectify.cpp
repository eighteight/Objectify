
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
    void DoRecursion(BaseObject *op, BaseObject *child, GeDynamicArray<Vector> &points, Matrix ml);
    int ksearchNeighbors;
    float gp3SearchRadius;
    int gp3MaxNeighbors;
    float gp3Mu;
    GeDynamicArray<GeDynamicArray<Vector> > splineAtPoint;
    Triangulator tri;
    LONG prevFrame;
public:
    virtual BaseObject* GetVirtualObjects(BaseObject *op, HierarchyHelp *hh);
    virtual Bool Init(GeListNode *node);
    
    static NodeData *Alloc(void) { return gNew Objectify; }
};

Bool Objectify::Init(GeListNode *node)
{
    BaseObject    *op   = (BaseObject*)node;
    BaseContainer *data = op->GetDataInstance();
    
    data->SetReal(TRIANGULATION_MAX_SEARCH_RADIUS,30.);

    GePrint("Objectify by http://twitter.com/eight_io for Cinema 4D r14");
    prevFrame = 0;
    return TRUE;
}

BaseObject *Objectify::GetVirtualObjects(BaseObject *op, HierarchyHelp *hh)
{
    BaseDocument *doc = op->GetDocument();
    BaseContainer *data = op->GetDataInstance();
    BaseObject* obj = (BaseObject*)data->GetLink(CTT_OBJECT_LINK,doc,Obase);
    LONG crntFrame = doc->GetTime().GetFrame(doc->GetFps());
    if (!obj) return NULL;
    
    if (obj->GetType() != Ospline){
        return NULL;
    }
    // start new list
    op->NewDependenceList();
    
    // check cache for validity and check master object for changes
    Bool dirty = op->CheckCache(hh) || op->IsDirty(DIRTYFLAGS_DATA);

    // if child list has been modified
    if (!dirty) dirty = !op->CompareDependenceList();
    
    // mark child objects as processed
    op->TouchDependenceList();

    dirty = dirty || (prevFrame != crntFrame);
    prevFrame = crntFrame;
    // if no change has been detected, return original cache
    if (!dirty) return op->GetCache(hh);
    
    SplineObject* spline = (SplineObject*) obj;
    
    AutoAlloc<SplineHelp> splineHelp;
    
    StatusSetText("Collecting points");
    float positioAlongSpline = data->GetReal(POSITION_ALONG_SPLINE,0.5);
    positioAlongSpline /= 10000.0f;
    positioAlongSpline = positioAlongSpline > 1.0 ? 1.0: positioAlongSpline;
    positioAlongSpline = positioAlongSpline < 0.0 ? 0.0: positioAlongSpline;
    cout<<positioAlongSpline<<endl;
    vector<vector<float> > points;
    for (LONG i = 0; i < spline->GetSegmentCount(); i++){
        Vector p = spline->GetSplinePoint(positioAlongSpline, i);
        vector<float> point;
        point.push_back(p.x);
        point.push_back(p.y);
        point.push_back(p.z);
        points.push_back(point);
    }
    
    StatusSetText("Collected "+LongToString(points.size())+" points");

    ksearchNeighbors= data->GetLong(TRIANGULATION_MAX_NEIGHBORS, 100);
    gp3SearchRadius = data->GetReal(TRIANGULATION_MAX_SEARCH_RADIUS,30.);
    gp3MaxNeighbors = data->GetLong(KSEARCH_NEIGHBORS, 20);

    gp3Mu = data->GetReal(KSEARCH_MU, 0.5);

    vector<vector<float> > surfacePoints;
    vector<vector<int> > triIndxs;
    StatusSetBar(0);
    StatusSetText("Triangulating");
    
    bool useMls = data->GetBool(USE_MLS, false);

    tri.computeSurface(points, surfacePoints, triIndxs, ksearchNeighbors, gp3SearchRadius, gp3MaxNeighbors, gp3Mu, useMls);

    StatusSetBar(100);
    StatusSetText("Got "+LongToString(triIndxs.size())+" triangles");

    if (triIndxs.size() == 0){
        return NULL;
    }
    
    PolygonObject* myPoly = PolygonObject::Alloc(surfacePoints.size(),triIndxs.size());
    Vector* ppoints = myPoly->GetPointW();
    
    vector<float> sp;
    for (int t = 0; t < surfacePoints.size()-2; t += 3) {
        sp = surfacePoints[t];
        ppoints[t+0] = Vector(sp[0],sp[1],sp[2]);
        
        sp = surfacePoints[t+1];
        ppoints[t+1] = Vector(sp[0],sp[1],sp[2]);
        
        sp = surfacePoints[t+2];
        ppoints[t+2] = Vector(sp[0],sp[1],sp[2]);
    }
    
    for (int t = 0; t < triIndxs.size(); t ++) {
        myPoly->GetPolygonW()[t] = CPolygon(triIndxs[t][0], triIndxs[t][1], triIndxs[t][2], triIndxs[t][2]);
    }
    
    StatusClear();
    myPoly->Message(MSG_UPDATE);
    return ToPoly(myPoly);

    BaseThread* bt=hh->GetThread();
    BaseObject* main = BaseObject::Alloc(Onull);

    SplineObject* emptySpline = SplineObject::Alloc(0, SPLINETYPE_LINEAR);
    ModelingCommandData mcd;
    
    mcd.doc = doc;
    
    mcd.op = emptySpline;
    
    if(!SendModelingCommand(MCOMMAND_JOIN, mcd)){
        return NULL;
    }
Error:

    return NULL;
}

// unique ID obtained from www.plugincafe.com



Bool RegisterObjectify(void)
{
	return RegisterObjectPlugin(ID_OBJECTIFY,GeLoadString(IDS_OBJECTIFY),OBJECT_GENERATOR|OBJECT_INPUT|OBJECT_ISSPLINE|OBJECT_CALL_ADDEXECUTION,Objectify::Alloc,"Octobjectify",AutoBitmap("tsp.tif"),0);
}
