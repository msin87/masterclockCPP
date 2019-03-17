#include "IDialog.h"

IDialog::~IDialog()
{
	WM_DeleteWindow(hWin);
}

void IDialog::createAndShow()
{

	hWin = GUI_CreateDialogBox(aDialogCreate, (sizeof(aDialogCreate) / sizeof(aDialogCreate[0])), callBack, hParrent, x0, y0);
	hMenuStack.push(hWin);
}

void IDialog::hide()
{
	WM_HideWindow(hWin);
}

void IDialog::destroy()
{
	WM_DeleteWindow(hWin);
	hWin = 0;
}