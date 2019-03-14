#pragma once
#include "functional"
#include "DIALOG.h"

class IDialog
{
	WM_HWIN hWin;
	const WM_HWIN hParrent;
	static WM_HWIN hWinPrev;
	WM_CALLBACK *callBack;
	const GUI_WIDGET_CREATE_INFO *aDialogCreate;
	uint16_t x0, y0;

  public:
	IDialog(const GUI_WIDGET_CREATE_INFO *_aDialogCreate, WM_CALLBACK *_callBack, const WM_HWIN _hParrent, uint16_t _x0, uint16_t _y0) : aDialogCreate(_aDialogCreate), callBack(_callBack), hParrent(_hParrent), x0(_x0), y0(_y0){};
	~IDialog();
	void createDialogBox();
};
