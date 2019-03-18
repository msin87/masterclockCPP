#include "IDialog.h"

IDialog::~IDialog()
{
	WM_DeleteWindow(hWin);
}

void IDialog::createAndShow()
{

	hMenuStack.push(GUI_CreateDialogBox(aDialogCreate, (sizeof(aDialogCreate) / sizeof(aDialogCreate[0])), callBack, hParrent, x0, y0));
	visible = true;
}

void IDialog::hide()
{
	if (hMenuStack.top() == hWin)
	{
		WM_HideWindow(hWin);
		visible = false;
	}
}

void IDialog::show()
{
	if (hWin && hMenuStack.top != hWin && !visible)
	{
		hMenuStack.push(hWin);
		WM_ShowWindow(hWin);
	}
}

void IDialog::destroy()
{
	if (hMenuStack.top == hWin)
	{
		hMenuStack.pop();
		WM_DeleteWindow(hWin);
		hWin = 0;
		visible = false;
	}
}