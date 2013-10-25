
#include "c4d.h"
#include "c4d_symbols.h"
#include "c4d_tools.h"
#include "lib_splinehelp.h"
#include "ge_dynamicarray.h"
#include "octobjectify.h"
#include "kd_tree.h"
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
    GePrint("Splinify by http://twitter.com/eight_io for Cinema 4D r14");
    return TRUE;
}

void Objectify::DoRecursion(BaseObject *op, BaseObject *child, GeDynamicArray<Vector> &points, Matrix ml)
{
    BaseObject *tp;
    if (child){
        tp = child->GetDeformCache();
        ml = ml * child->GetMl();
        if (tp){
            DoRecursion(op,tp,points,ml);
        }
        else{
            tp = child->GetCache(NULL);
            if (tp){
                DoRecursion(op,tp,points,ml);
            }
            else{
                
                if (!child->GetBit(BIT_CONTROLOBJECT)){
                    if (child->IsInstanceOf(Opoint)){
                        PointObject * pChild = ToPoint(child);
                        LONG pcnt = pChild->GetPointCount();
                        const Vector *childVerts = pChild->GetPointR();
                        for(LONG i=0;i<pcnt;i++){
                            points.Push(childVerts[i] * ml);
                        }
                    }
                }
            }
        }
        for (tp = child->GetDown(); tp; tp=tp->GetNext()){
            DoRecursion(op,tp,points,ml);
        }
    }
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
    
    vector<Triangle> triangles = tri.triangulate(points);

    PolygonObject* myPoly = PolygonObject::Alloc( triangles.size()*3,triangles.size());
    Vector* ppoints = myPoly->GetPointW();
    for (int t = 0; t < triangles.size(); t++) {
        Triangle tr = triangles[t];
        std::cout << tr << std::endl;
        ppoints[t+0] = 100.0*Vector(tr.a[0],tr.a[1],tr.a[2]);
        ppoints[t+1] = 100.0*Vector(tr.b[0],tr.b[1],tr.b[2]);
        ppoints[t+2] = 100.0*Vector(tr.c[0],tr.c[1],tr.c[2]);
        myPoly->GetPolygonW()[t] = CPolygon(tr.index[0],tr.index[1],tr.index[2]);
    }

    for(LONG i=0; i < myPoly->GetPolygonCount()-1; i++){
        Triangle t = triangles[i];
        //myPoly->GetPolygonW()[i] = CPolygon(t.index[0],t.index[1],t.index[2]);
        //myPoly->GetPolygonW()[i] = CPolygon(i,i+1,i+2);
    }
    return myPoly;
    
    
   BaseObject* chld = NULL;
    LONG trck = 0;

    // if child list has been modified
    if (!dirty) dirty = !op->CompareDependenceList();
    
    // mark child objects as processed
    op->TouchDependenceList();
    
    // if no change has been detected, return original cache
    if (!dirty) return op->GetCache(hh);
    
    Real maxSeg = data->GetReal(CTTSPOBJECT_MAXSEG,30.);
    Bool relativeMaxSeg  = data->GetBool(CTTSPOBJECT_REL,TRUE);
    
    LONG splineInterpolation = data->GetLong(SPLINEOBJECT_INTERPOLATION);
    
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
    
    return ToSpline(mcd.result->GetIndex(0L));
    
    //        BaseObject *newOp = static_cast<BaseObject*>(mcd.result->GetIndex(0));
    
    
    /////////////////
    GeDynamicArray<LONG> segments;
    SplineObject        *pp= SplineObject::Alloc(0,SPLINETYPE_LINEAR);
    if (!pp) return NULL;
    pp->ResizeObject(100,segments.GetCount());
    
    Segment* seg = pp->GetSegmentW();
    for(LONG i=0;i<segments.GetCount();i++){
        seg[i].cnt = segments[i];
        seg[i].closed = FALSE;
    }
    
    Vector *padr=pp->GetPointW();
    VLONG j = 0;
    for (LONG i = 0; i < splineAtPoint.GetCount(); i++){
        for (LONG k = 0; k < splineAtPoint[i].GetCount(); k++){
            padr[j++] = splineAtPoint[i][k];
        }
    }
    
    //////////////////
    
    main->Message(MSG_UPDATE);
    prvsFrame = crntFrame;
    StatusClear();
    return pp;
    //return main;
Error:

    return NULL;
}

// unique ID obtained from www.plugincafe.com



Bool RegisterObjectify(void)
{
	return RegisterObjectPlugin(ID_OBJECTIFY,GeLoadString(IDS_OBJECTIFY),OBJECT_GENERATOR|OBJECT_INPUT|OBJECT_ISSPLINE|OBJECT_CALL_ADDEXECUTION,Objectify::Alloc,"Octobjectify",AutoBitmap("tsp.tif"),0);
}
