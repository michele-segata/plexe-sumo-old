#ifndef GUIGLObjectPopupMenu_h
#define GUIGLObjectPopupMenu_h
//---------------------------------------------------------------------------//
//                        GUIGLObjectPopupMenu.h -
//  The popup menu which is displayed when pressing the right mouse button over
//  a gl-object
//                           -------------------
//  project              : SUMO - Simulation of Urban MObility
//  begin                : Sept 2002
//  copyright            : (C) 2002 by Daniel Krajzewicz
//  organisation         : IVF/DLR http://ivf.dlr.de
//  email                : Daniel.Krajzewicz@dlr.de
//---------------------------------------------------------------------------//

//---------------------------------------------------------------------------//
//
//   This program is free software; you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or
//   (at your option) any later version.
//
//---------------------------------------------------------------------------//
// $Log$
// Revision 1.6  2006/10/12 07:57:14  dkrajzew
// added the possibility to copy an artefact's (gl-object's) name to clipboard (windows)
//
// Revision 1.5  2006/04/18 08:08:21  dkrajzew
// added Danilot Tete-Boyoms poi-interaction
//
// Revision 1.4  2005/10/07 11:45:32  dkrajzew
// THIRD LARGE CODE RECHECK: patched problems on Linux/Windows configs
//
// Revision 1.3  2005/09/15 12:19:44  dkrajzew
// LARGE CODE RECHECK
//
// Revision 1.2  2005/05/04 09:19:27  dkrajzew
// adding of surrounding lanes to selection
//
// Revision 1.1  2004/11/23 10:38:31  dkrajzew
// debugging
//
// Revision 1.1  2004/10/22 12:50:50  dksumo
// initial checkin into an internal, standalone SUMO CVS
//
// Revision 1.2  2004/07/02 08:25:32  dkrajzew
// possibility to manipulate objects added
//
// Revision 1.1  2004/03/19 12:41:13  dkrajzew
// porting to FOX
//
// Revision 1.3  2003/11/12 14:09:39  dkrajzew
// clean up after recent changes; comments added
//
// Revision 1.2  2003/07/30 08:49:26  dkrajzew
// changed the responsibility of a GLObject
//
// Revision 1.1  2003/06/06 10:24:36  dkrajzew
// new subfolder holding popup-menus was added due to link-dependencies under
//  linux; GUIGLObjectPopupMenu*-classes were moved to "popup"
//
// Revision 1.2  2003/06/05 11:37:31  dkrajzew
// class templates applied
//
/* =========================================================================
 * compiler pragmas
 * ======================================================================= */
#pragma warning(disable: 4786)


/* =========================================================================
 * included modules
 * ======================================================================= */
#ifdef HAVE_CONFIG_H
#ifdef WIN32
#include <windows_config.h>
#else
#include <config.h>
#endif
#endif // HAVE_CONFIG_H

#include <vector>
#include <fx.h>


/* =========================================================================
 * class declarations
 * ======================================================================= */
class GUISUMOAbstractView;
class GUIGlObject;
class GUIMainWindow;


/* =========================================================================
 * class definitions
 * ======================================================================= */
/**
 * @class GUIGLObjectPopupMenu
 */
class GUIGLObjectPopupMenu : public FXMenuPane
{
    // FOX-declarations
    FXDECLARE(GUIGLObjectPopupMenu)
public:
    /// Constructor
    GUIGLObjectPopupMenu(GUIMainWindow &app,
        GUISUMOAbstractView &parent, GUIGlObject &o);

    /// Destructor
    virtual ~GUIGLObjectPopupMenu();

public:
    /// Called if the assigned objects shall be centered
    long onCmdCenter(FXObject*,FXSelector,void*);

    /// Called if the name shall be copied to clipboard
    long onCmdCopyName(FXObject*,FXSelector,void*);

    /// Called if the typed name shall be copied to clipboard
    long onCmdCopyTypedName(FXObject*,FXSelector,void*);

    /// Called if the parameter of this object shall be shown
    long onCmdShowPars(FXObject*,FXSelector,void*);

    /// Called if the object shall be added to the list of selected objects
    long onCmdAddSelected(FXObject*,FXSelector,void*);

    /// Called if the object shall be removed from the list of selected objects
    long onCmdRemoveSelected(FXObject*,FXSelector,void*);

    // Called if the object's consecutives shall be added to the list of selected objects
    long onCmdAddSuccessorsSelected(FXObject*,FXSelector,void*);

    /// Called if the assigned objects shall be centered
	long onCmdRename(FXObject*,FXSelector,void*);

	/// Called if the assigned objects shall be moved
	long onCmdMoveTo(FXObject*,FXSelector,void*);

	/// Called if the assigned objects shall be recolored
	long onCmdChangeCol(FXObject*,FXSelector,void*);

	/// Called if the assigned objects shall be changed in typ
	long onCmdChangeTyp(FXObject*,FXSelector,void*);

	/// Called if the assigned objects shall be deleted
	long onCmdDelete(FXObject*,FXSelector,void*);


protected:
    /// The parent window
    GUISUMOAbstractView *myParent; // !!! needed?

    /// The object that belongs to this popup-menu
    GUIGlObject *myObject;

    /// The main application
    GUIMainWindow *myApplication;

protected:
    /// FOX needs this
    GUIGLObjectPopupMenu() { }

};


/**************** DO NOT DEFINE ANYTHING AFTER THE INCLUDE *****************/

#endif

// Local Variables:
// mode:C++
// End:

