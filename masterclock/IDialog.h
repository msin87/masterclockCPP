
#include <stack>
#include "DIALOG.h"
#include <stdint.h>

enum MenuState
{
	MENU_STATE_MAIN,
	MENU_STATE_TIMESETUP,
	MENU_STATE_TIMEDATESETUP,
	MENU_STATE_TIMECALIBRATION,
	MENU_STATE_LINE1SETUP,
	MENU_STATE_LINE2SETUP,
	MENU_STATE_LINE3SETUP,
	MENU_STATE_LINE4SETUP,
	MENU_STATE_LINE1SETUP_PULSE,
	MENU_STATE_LINE2SETUP_PULSE,
	MENU_STATE_LINE3SETUP_PULSE,
	MENU_STATE_LINE4SETUP_PULSE,
	MENU_STATE_TIME_SUMWIN,
	MENU_STATE_PASSWORD,
	MENU_STATE_SETTINGS,
	MENU_STATE_TOUCHCALIBRATION
};
class IDialog
{
	WM_HWIN hWin;
	const WM_HWIN hParrent;
	static std::stack<WM_HWIN> hMenuStack;
	WM_CALLBACK *const callBack;
	const GUI_WIDGET_CREATE_INFO *aDialogCreate;
	uint16_t x0, y0;
	bool visible;

  public:
	IDialog(const GUI_WIDGET_CREATE_INFO *_aDialogCreate, WM_CALLBACK *const _callBack, const WM_HWIN _hParrent, uint16_t _x0, uint16_t _y0) : aDialogCreate(_aDialogCreate), callBack(_callBack), hParrent(_hParrent), x0(_x0), y0(_y0){};
	~IDialog();
	void createAndShow();
	void hide();
	void show();
	void destroy();
};
