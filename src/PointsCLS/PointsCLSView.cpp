
// PointsCLSView.cpp : implementation of the CPointsCLSView class
//

#include "stdafx.h"
// SHARED_HANDLERS can be defined in an ATL project implementing preview, thumbnail
// and search filter handlers and allows sharing of document code with that project.
#ifndef SHARED_HANDLERS
#include "PointsCLS.h"
#endif

#include "PointsCLSDoc.h"
#include "PointsCLSView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CPointsCLSView

IMPLEMENT_DYNCREATE(CPointsCLSView, CView)

BEGIN_MESSAGE_MAP(CPointsCLSView, CView)
	// Standard printing commands
	ON_COMMAND(ID_FILE_PRINT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_DIRECT, &CView::OnFilePrint)
	ON_COMMAND(ID_FILE_PRINT_PREVIEW, &CPointsCLSView::OnFilePrintPreview)
	ON_WM_CONTEXTMENU()
	ON_WM_RBUTTONUP()
END_MESSAGE_MAP()

// CPointsCLSView construction/destruction

CPointsCLSView::CPointsCLSView()
{
	// TODO: add construction code here

}

CPointsCLSView::~CPointsCLSView()
{
}

BOOL CPointsCLSView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return CView::PreCreateWindow(cs);
}

// CPointsCLSView drawing

void CPointsCLSView::OnDraw(CDC* /*pDC*/)
{
	CPointsCLSDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO: add draw code for native data here
}


// CPointsCLSView printing


void CPointsCLSView::OnFilePrintPreview()
{
#ifndef SHARED_HANDLERS
	AFXPrintPreview(this);
#endif
}

BOOL CPointsCLSView::OnPreparePrinting(CPrintInfo* pInfo)
{
	// default preparation
	return DoPreparePrinting(pInfo);
}

void CPointsCLSView::OnBeginPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add extra initialization before printing
}

void CPointsCLSView::OnEndPrinting(CDC* /*pDC*/, CPrintInfo* /*pInfo*/)
{
	// TODO: add cleanup after printing
}

void CPointsCLSView::OnRButtonUp(UINT /* nFlags */, CPoint point)
{
	ClientToScreen(&point);
	OnContextMenu(this, point);
}

void CPointsCLSView::OnContextMenu(CWnd* /* pWnd */, CPoint point)
{
#ifndef SHARED_HANDLERS
	theApp.GetContextMenuManager()->ShowPopupMenu(IDR_POPUP_EDIT, point.x, point.y, this, TRUE);
#endif
}


// CPointsCLSView diagnostics

#ifdef _DEBUG
void CPointsCLSView::AssertValid() const
{
	CView::AssertValid();
}

void CPointsCLSView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CPointsCLSDoc* CPointsCLSView::GetDocument() const // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CPointsCLSDoc)));
	return (CPointsCLSDoc*)m_pDocument;
}
#endif //_DEBUG


// CPointsCLSView message handlers
