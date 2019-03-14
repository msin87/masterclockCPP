#include "IDialog.h"

IDialog::~IDialog()
{
	WM_DeleteWindow(hWin);
}

void IDialog::createDialogBox()
{
	
	hWin = GUI_CreateDialogBox(aDialogCreate, (sizeof(aDialogCreate)/sizeof(aDialogCreate[0])), callBack, hParrent, x0, y0);
}
