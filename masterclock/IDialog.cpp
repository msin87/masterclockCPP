#include "IDialog.h"

IDialog::~IDialog()
{
	WM_DeleteWindow(hWin);
}

void IDialog::createDialogBox()
{
	hWin = GUI_CreateDialogBox(const GUI_WIDGET_CREATE_INFO *paWidget, int NumWidgets, WM_CALLBACK *cb, WM_HWIN hParent, int x0, int y0);
}
