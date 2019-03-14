#pragma once
#include "functional"
#include "DIALOG.h"

class IDialog
{
	WM_HWIN hWin;
	static WM_HWIN hWinPrev;
	const std::function<void(WM_MESSAGE *pMsg)> callBack;
	const GUI_WIDGET_CREATE_INFO *aDialogCreate;

  public:
	IDialog(const GUI_WIDGET_CREATE_INFO *_aDialogCreate, const std::function<void(WM_MESSAGE *pMsg)> _callBack) : aDialogCreate(_aDialogCreate), callBack(_callBack){};
	~IDialog();
	void createDialogBox();
};
