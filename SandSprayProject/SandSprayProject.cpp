//SandSprayProject

// Mandatory UF Includes
#include <uf.h>
#include <uf_object_types.h>
#include <uf_layer.h>
// Internal Includes
#include <NXOpen/ListingWindow.hxx>
#include <NXOpen/NXMessageBox.hxx>
#include <NXOpen/UI.hxx>

// Internal+External Includes
#include <NXOpen/Annotations.hxx>
#include <NXOpen/Assemblies_Component.hxx>
#include <NXOpen/Assemblies_ComponentAssembly.hxx>
#include <NXOpen/Body.hxx>
#include <NXOpen/BodyCollection.hxx>
#include <NXOpen/Face.hxx>
#include <NXOpen/Line.hxx>
#include <NXOpen/NXException.hxx>
#include <NXOpen/NXObject.hxx>
#include <NXOpen/Part.hxx>
#include <NXOpen/PartCollection.hxx>
#include <NXOpen/Session.hxx>
#include <NXOpen/ModelingView.hxx>
#include <NXOpen/ModelingViewCollection.hxx>
// Std C++ Includes
#include <iostream>
#include <sstream>


//dlxinclude
#include "GeometricProcessFaceClassification.hpp"
#include "SprayCurveBuilder.hpp"
#include "SprayEffectEvalutatorDialog.hpp"
#include "SpraySequencerandCSVOutputDialog.hpp"

using namespace NXOpen;
using std::string;
using std::exception;
using std::stringstream;
using std::endl;
using std::cout;
using std::cerr;

//------------------------------------------------------------------------------
// Open C error handling
//------------------------------------------------------------------------------
#define UF_CALL(X) (report_error( __FILE__, __LINE__, #X, (X)))
int report_error(const char *file, int line, const char *call, int code)
{
    if (code) 
	{

		stringstream errmsg;
		errmsg << "Error " << code << " in " << file << " at line " << line << endl;
		errmsg << call << endl;
		UI::GetUI()->NXMessageBox()->Show("Open C Error", NXOpen::NXMessageBox::DialogTypeError, errmsg.str().c_str());
		throw NXOpen::NXException::Create(code);
	}
    return(code);
}

void GeometricProcessFaceClassification_entry();
void SprayCurveBuilder_entry();
void SprayEffectEvalutatorDialog_entry();
void SpraySequencerandCSVOutputDialog_entry();

//------------------------------------------------------------------------------
// NXOpen c++ test class 
//------------------------------------------------------------------------------
class SandSprayProject
{
    // class members
public:
    static Session *theSession;
    static UI *theUI;

    SandSprayProject();
    ~SandSprayProject();

	void do_it(char* parm, int* returnCode, int rlen);
	void print(const NXString &);
	void print(const string &);
	void print(const char*);

private:
	BasePart *workPart, *displayPart;
	NXMessageBox *mb;
	ListingWindow *lw;
	LogFile *lf;
};

//------------------------------------------------------------------------------
// Initialize static variables
//------------------------------------------------------------------------------
Session *(SandSprayProject::theSession) = NULL;
UI *(SandSprayProject::theUI) = NULL;

//------------------------------------------------------------------------------
// Constructor 
//------------------------------------------------------------------------------
SandSprayProject::SandSprayProject()
{
	// Initialize the Open C API environment */
	UF_CALL( UF_initialize() );

	// Initialize the NX Open C++ API environment
	SandSprayProject::theSession = NXOpen::Session::GetSession();
	SandSprayProject::theUI = UI::GetUI();
	mb = theUI->NXMessageBox();
	lw = theSession->ListingWindow();
	lf = theSession->LogFile();

    workPart = theSession->Parts()->BaseWork();
	displayPart = theSession->Parts()->BaseDisplay();
	
}

//------------------------------------------------------------------------------
// Destructor
//------------------------------------------------------------------------------
SandSprayProject::~SandSprayProject()
{
	UF_CALL( UF_terminate() );
}

//------------------------------------------------------------------------------
// Print string to listing window or stdout
//------------------------------------------------------------------------------
void SandSprayProject::print(const NXString &msg)
{
	if(! lw->IsOpen() ) lw->Open();
	lw->WriteLine(msg);
}
void SandSprayProject::print(const string &msg)
{
	if(! lw->IsOpen() ) lw->Open();
	lw->WriteLine(msg);
}
void SandSprayProject::print(const char * msg)
{
	if(! lw->IsOpen() ) lw->Open();
	lw->WriteLine(msg);
}




//------------------------------------------------------------------------------
// Do something
//------------------------------------------------------------------------------
void SandSprayProject::do_it(char* parm, int* returnCode, int rlen)
{

	if (strcmp(parm, "FACE_PROCESS_BTN") == 0)
	{
		GeometricProcessFaceClassification_entry();
	}
	else if (strcmp(parm, "REGION_BUILDER_BUTTON") == 0)
	{
		NXOpen::Session* theSession = NXOpen::Session::GetSession();
		NXOpen::Part* workPart(theSession->Parts()->Work());
		NXOpen::Part* displayPart(theSession->Parts()->Display());
		workPart->ModelingViews()->WorkView()->SetRenderingStyle(NXOpen::View::RenderingStyleTypeWireframeWithDimEdges);
		UF_LAYER_set_status(80, UF_LAYER_WORK_LAYER);
		SprayCurveBuilder_entry();
	}
	else if (strcmp(parm, "REGION_CONNECTOR_BUTTON") == 0)
	{
		NXOpen::Session* theSession = NXOpen::Session::GetSession();
		NXOpen::Part* workPart(theSession->Parts()->Work());
		NXOpen::Part* displayPart(theSession->Parts()->Display());
		workPart->ModelingViews()->WorkView()->SetRenderingStyle(NXOpen::View::RenderingStyleTypeWireframeWithDimEdges);
		
		SpraySequencerandCSVOutputDialog_entry();
	}
	else if (strcmp(parm, "REGION_EVALUATOR_BUTTON") == 0)
	{
		NXOpen::Session* theSession = NXOpen::Session::GetSession();
		NXOpen::Part* workPart(theSession->Parts()->Work());
		NXOpen::Part* displayPart(theSession->Parts()->Display());
		workPart->ModelingViews()->WorkView()->SetRenderingStyle(NXOpen::View::RenderingStyleTypeWireframeWithDimEdges);

		SprayEffectEvalutatorDialog_entry();
	}

	
}

//------------------------------------------------------------------------------
// Entry point(s) for unmanaged internal NXOpen C/C++ programs
//------------------------------------------------------------------------------
//  Explicit Execution
extern "C" DllExport void ufusr( char *parm, int *returnCode, int rlen )
{
    try
    {
		// Create NXOpen C++ class instance
		SandSprayProject *theSandSprayProject;
		theSandSprayProject = new SandSprayProject();
		theSandSprayProject->do_it(parm,returnCode, rlen);
		delete theSandSprayProject;
	}
    catch (const NXException& e1)
    {
		UI::GetUI()->NXMessageBox()->Show("NXException", NXOpen::NXMessageBox::DialogTypeError, e1.Message());
    }
	catch (const exception& e2)
    {
		UI::GetUI()->NXMessageBox()->Show("Exception", NXOpen::NXMessageBox::DialogTypeError, e2.what());
    }
	catch (...)
    {
		UI::GetUI()->NXMessageBox()->Show("Exception", NXOpen::NXMessageBox::DialogTypeError, "Unknown Exception.");
    }
}


//------------------------------------------------------------------------------
// Unload Handler
//------------------------------------------------------------------------------
extern "C" DllExport int ufusr_ask_unload()
{
	// Unloads the image when the application completes
	return (int)Session::LibraryUnloadOptionImmediately;	
	
}
//------------------------------------------------------------------------------
// DLX entry
//------------------------------------------------------------------------------
void GeometricProcessFaceClassification_entry()
{
	GeometricProcessFaceClassification* theGeometricProcessFaceClassification = NULL;
	try
	{
		theGeometricProcessFaceClassification = new GeometricProcessFaceClassification();
		// The following method shows the dialog immediately
		theGeometricProcessFaceClassification->Launch();
	}
	catch (exception& ex)
	{
		//---- Enter your exception handling code here -----
		GeometricProcessFaceClassification::theUI->NXMessageBox()->Show("Block Styler", NXOpen::NXMessageBox::DialogTypeError, ex.what());
	}
	if (theGeometricProcessFaceClassification != NULL)
	{
		delete theGeometricProcessFaceClassification;
		theGeometricProcessFaceClassification = NULL;
	}
}

void SprayCurveBuilder_entry()
{
	SprayCurveBuilder* theSprayCurveBuilder = NULL;
	try
	{
		theSprayCurveBuilder = new SprayCurveBuilder();
		// The following method shows the dialog immediately
		theSprayCurveBuilder->Launch();
	}
	catch (exception& ex)
	{
		//---- Enter your exception handling code here -----
		SprayCurveBuilder::theUI->NXMessageBox()->Show("Block Styler", NXOpen::NXMessageBox::DialogTypeError, ex.what());
	}
	if (theSprayCurveBuilder != NULL)
	{
		delete theSprayCurveBuilder;
		theSprayCurveBuilder = NULL;
	}
}


void SprayEffectEvalutatorDialog_entry()
{
	SprayEffectEvalutatorDialog* theSprayEffectEvalutatorDialog = NULL;
	try
	{
		theSprayEffectEvalutatorDialog = new SprayEffectEvalutatorDialog();
		// The following method shows the dialog immediately
		theSprayEffectEvalutatorDialog->Launch();
	}
	catch (exception& ex)
	{
		//---- Enter your exception handling code here -----
		SprayEffectEvalutatorDialog::theUI->NXMessageBox()->Show("Block Styler", NXOpen::NXMessageBox::DialogTypeError, ex.what());
	}
	if (theSprayEffectEvalutatorDialog != NULL)
	{
		delete theSprayEffectEvalutatorDialog;
		theSprayEffectEvalutatorDialog = NULL;
	}
}

void SpraySequencerandCSVOutputDialog_entry()
{
	SpraySequencerandCSVOutputDialog* theSpraySequencerandCSVOutputDialog = NULL;
	try
	{
		theSpraySequencerandCSVOutputDialog = new SpraySequencerandCSVOutputDialog();
		// The following method shows the dialog immediately
		theSpraySequencerandCSVOutputDialog->Launch();
	}
	catch (exception& ex)
	{
		//---- Enter your exception handling code here -----
		SpraySequencerandCSVOutputDialog::theUI->NXMessageBox()->Show("Block Styler", NXOpen::NXMessageBox::DialogTypeError, ex.what());
	}
	if (theSpraySequencerandCSVOutputDialog != NULL)
	{
		delete theSpraySequencerandCSVOutputDialog;
		theSpraySequencerandCSVOutputDialog = NULL;
	}
}


