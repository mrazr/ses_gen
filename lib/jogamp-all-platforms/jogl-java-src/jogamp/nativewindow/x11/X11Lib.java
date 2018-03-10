/* !---- DO NOT EDIT: This file autogenerated by com/jogamp/gluegen/JavaEmitter.java on Sat Oct 10 03:10:17 CEST 2015 ----! */

package jogamp.nativewindow.x11;

import java.nio.*;
import java.util.*;
import com.jogamp.nativewindow.util.Point;
import com.jogamp.common.util.Bitfield;
import com.jogamp.gluegen.runtime.*;
import com.jogamp.common.os.*;
import com.jogamp.common.nio.*;
import java.nio.*;

public class X11Lib {

  /** CType: int */
  public static final int VisualNoMask = 0x0;
  /** CType: int */
  public static final int PictFormatBlueMask = ( 0x1 << 0x8 );
  /** CType: int */
  public static final int VisualScreenMask = 0x2;
  /** CType: int */
  public static final int PictFormatRed = ( 0x1 << 0x3 );
  /** CType: int */
  public static final int PictFormatType = ( 0x1 << 0x1 );
  /** CType: int */
  public static final int VisualGreenMaskMask = 0x20;
  /** CType: int */
  public static final int PictFormatRedMask = ( 0x1 << 0x4 );
  /** CType: int */
  public static final int VisualBlueMaskMask = 0x40;
  /** CType: int */
  public static final int PictFormatGreen = ( 0x1 << 0x5 );
  /** CType: int */
  public static final int VisualAllMask = 0x1ff;
  /** CType: int */
  public static final int PictFormatAlpha = ( 0x1 << 0x9 );
  /** CType: int */
  public static final int VisualIDMask = 0x1;
  /** CType: int */
  public static final int VisualDepthMask = 0x4;
  /** CType: int */
  public static final int PictFormatColormap = ( 0x1 << 0xb );
  /** CType: int */
  public static final int PictFormatDepth = ( 0x1 << 0x2 );
  /** CType: int */
  public static final int VisualColormapSizeMask = 0x80;
  /** CType: int */
  public static final int PictFormatID = ( 0x1 << 0x0 );
  /** CType: int */
  public static final int VisualBitsPerRGBMask = 0x100;
  /** CType: int */
  public static final int PictFormatBlue = ( 0x1 << 0x7 );
  /** CType: int */
  public static final int VisualRedMaskMask = 0x10;
  /** CType: int */
  public static final int PictFormatAlphaMask = ( 0x1 << 0xa );
  /** CType: int */
  public static final int PictFormatGreenMask = ( 0x1 << 0x6 );
  /** CType: int */
  public static final int VisualClassMask = 0x8;

  /** Interface to C language function: <br> <code>void *  XineramaGetLibHandle()</code><br>   */
  public static native long XineramaGetLibHandle();

  /** Interface to C language function: <br> <code>Bool XineramaReleaseLibHandle(void *  xineramaLibHandle)</code><br>   */
  public static native boolean XineramaReleaseLibHandle(long xineramaLibHandle);

  /** Interface to C language function: <br> <code>void *  XineramaGetQueryFunc(void *  xineramaLibHandle)</code><br>   */
  public static native long XineramaGetQueryFunc(long xineramaLibHandle);

  /** Interface to C language function: <br> <code>Bool XineramaIsEnabled(void *  xineramaQueryFunc, Display *  display)</code><br>   */
  public static native boolean XineramaIsEnabled(long xineramaQueryFunc, long display);

  /** Interface to C language function: <br> <code>intptr_t XSynchronize(Display *  display, Bool onoff)</code><br>   */
  public static native long XSynchronize(long display, boolean onoff);

  /** Interface to C language function: <br> <code>int XFlush(Display *  display)</code><br>   */
  public static native int XFlush(long display);

  /** Interface to C language function: <br> <code>int XSync(Display *  display, Bool discard)</code><br>   */
  public static native int XSync(long display, boolean discard);

  /** Interface to C language function: <br> <code>char *  XDisplayString(Display *  display)</code><br>   */
  public static native String XDisplayString(long display);

  /** Interface to C language function: <br> <code>Display *  XOpenDisplay(const char * )</code><br>   */
  public static native long XOpenDisplay(String arg0);

  /** Interface to C language function: <br> <code>int DefaultScreen(Display *  display)</code><br>   */
  public static native int DefaultScreen(long display);

  /** Interface to C language function: <br> <code>int ScreenCount(Display *  display)</code><br>   */
  public static native int ScreenCount(long display);

  /** Interface to C language function: <br> <code>Window RootWindow(Display *  display, int screen_number)</code><br>   */
  public static native long RootWindow(long display, int screen_number);

  /** Interface to C language function: <br> <code>Pixmap XCreatePixmap(Display * , Drawable, unsigned int, unsigned int, unsigned int)</code><br>   */
  public static native long XCreatePixmap(long arg0, long arg1, int arg2, int arg3, int arg4);

  /** Interface to C language function: <br> <code>int XFreePixmap(Display * , Pixmap)</code><br>   */
  public static native int XFreePixmap(long arg0, long arg1);

  /** Interface to C language function: <br> <code>int XFree(void * )</code><br>   */
  public static native int XFree(long arg0);

  /** Interface to C language function: <br> <code>Bool XF86VidModeGetGammaRampSize(Display *  display, int screen, int *  size)</code><br>
      @param size a direct or array-backed {@link java.nio.IntBuffer}   */
  public static boolean XF86VidModeGetGammaRampSize(long display, int screen, IntBuffer size)  {

    final boolean size_is_direct = Buffers.isDirect(size);
        return XF86VidModeGetGammaRampSize1(display, screen, size_is_direct ? size : Buffers.getArray(size), size_is_direct ? Buffers.getDirectBufferByteOffset(size) : Buffers.getIndirectBufferByteOffset(size), size_is_direct);
  }

  /** Entry point to C language function: <code>Bool XF86VidModeGetGammaRampSize(Display *  display, int screen, int *  size)</code><br>
      @param size a direct or array-backed {@link java.nio.IntBuffer}   */
  private static native boolean XF86VidModeGetGammaRampSize1(long display, int screen, Object size, int size_byte_offset, boolean size_is_direct);

  /** Interface to C language function: <br> <code>Bool XF86VidModeGetGammaRampSize(Display *  display, int screen, int *  size)</code><br>   */
  public static boolean XF86VidModeGetGammaRampSize(long display, int screen, int[] size, int size_offset)  {

    if(size != null && size.length <= size_offset)
      throw new RuntimeException("array offset argument \"size_offset\" (" + size_offset + ") equals or exceeds array length (" + size.length + ")");
        return XF86VidModeGetGammaRampSize1(display, screen, size, Buffers.SIZEOF_INT * size_offset, false);
  }

  /** Interface to C language function: <br> <code>Bool XF86VidModeGetGammaRamp(Display *  display, int screen, int size, unsigned short *  red_array, unsigned short *  green_array, unsigned short *  blue_array)</code><br>
      @param red_array a direct or array-backed {@link java.nio.ShortBuffer}
      @param green_array a direct or array-backed {@link java.nio.ShortBuffer}
      @param blue_array a direct or array-backed {@link java.nio.ShortBuffer}   */
  public static boolean XF86VidModeGetGammaRamp(long display, int screen, int size, ShortBuffer red_array, ShortBuffer green_array, ShortBuffer blue_array)  {

    final boolean red_array_is_direct = Buffers.isDirect(red_array);
    final boolean green_array_is_direct = Buffers.isDirect(green_array);
    final boolean blue_array_is_direct = Buffers.isDirect(blue_array);
        return XF86VidModeGetGammaRamp1(display, screen, size, red_array_is_direct ? red_array : Buffers.getArray(red_array), red_array_is_direct ? Buffers.getDirectBufferByteOffset(red_array) : Buffers.getIndirectBufferByteOffset(red_array), red_array_is_direct, green_array_is_direct ? green_array : Buffers.getArray(green_array), green_array_is_direct ? Buffers.getDirectBufferByteOffset(green_array) : Buffers.getIndirectBufferByteOffset(green_array), green_array_is_direct, blue_array_is_direct ? blue_array : Buffers.getArray(blue_array), blue_array_is_direct ? Buffers.getDirectBufferByteOffset(blue_array) : Buffers.getIndirectBufferByteOffset(blue_array), blue_array_is_direct);
  }

  /** Entry point to C language function: <code>Bool XF86VidModeGetGammaRamp(Display *  display, int screen, int size, unsigned short *  red_array, unsigned short *  green_array, unsigned short *  blue_array)</code><br>
      @param red_array a direct or array-backed {@link java.nio.ShortBuffer}
      @param green_array a direct or array-backed {@link java.nio.ShortBuffer}
      @param blue_array a direct or array-backed {@link java.nio.ShortBuffer}   */
  private static native boolean XF86VidModeGetGammaRamp1(long display, int screen, int size, Object red_array, int red_array_byte_offset, boolean red_array_is_direct, Object green_array, int green_array_byte_offset, boolean green_array_is_direct, Object blue_array, int blue_array_byte_offset, boolean blue_array_is_direct);

  /** Interface to C language function: <br> <code>Bool XF86VidModeGetGammaRamp(Display *  display, int screen, int size, unsigned short *  red_array, unsigned short *  green_array, unsigned short *  blue_array)</code><br>   */
  public static boolean XF86VidModeGetGammaRamp(long display, int screen, int size, short[] red_array, int red_array_offset, short[] green_array, int green_array_offset, short[] blue_array, int blue_array_offset)  {

    if(red_array != null && red_array.length <= red_array_offset)
      throw new RuntimeException("array offset argument \"red_array_offset\" (" + red_array_offset + ") equals or exceeds array length (" + red_array.length + ")");
    if(green_array != null && green_array.length <= green_array_offset)
      throw new RuntimeException("array offset argument \"green_array_offset\" (" + green_array_offset + ") equals or exceeds array length (" + green_array.length + ")");
    if(blue_array != null && blue_array.length <= blue_array_offset)
      throw new RuntimeException("array offset argument \"blue_array_offset\" (" + blue_array_offset + ") equals or exceeds array length (" + blue_array.length + ")");
        return XF86VidModeGetGammaRamp1(display, screen, size, red_array, Buffers.SIZEOF_SHORT * red_array_offset, false, green_array, Buffers.SIZEOF_SHORT * green_array_offset, false, blue_array, Buffers.SIZEOF_SHORT * blue_array_offset, false);
  }

  /** Interface to C language function: <br> <code>Bool XF86VidModeSetGammaRamp(Display *  display, int screen, int size, unsigned short *  red_array, unsigned short *  green_array, unsigned short *  blue_array)</code><br>
      @param red_array a direct or array-backed {@link java.nio.ShortBuffer}
      @param green_array a direct or array-backed {@link java.nio.ShortBuffer}
      @param blue_array a direct or array-backed {@link java.nio.ShortBuffer}   */
  public static boolean XF86VidModeSetGammaRamp(long display, int screen, int size, ShortBuffer red_array, ShortBuffer green_array, ShortBuffer blue_array)  {

    final boolean red_array_is_direct = Buffers.isDirect(red_array);
    final boolean green_array_is_direct = Buffers.isDirect(green_array);
    final boolean blue_array_is_direct = Buffers.isDirect(blue_array);
        return XF86VidModeSetGammaRamp1(display, screen, size, red_array_is_direct ? red_array : Buffers.getArray(red_array), red_array_is_direct ? Buffers.getDirectBufferByteOffset(red_array) : Buffers.getIndirectBufferByteOffset(red_array), red_array_is_direct, green_array_is_direct ? green_array : Buffers.getArray(green_array), green_array_is_direct ? Buffers.getDirectBufferByteOffset(green_array) : Buffers.getIndirectBufferByteOffset(green_array), green_array_is_direct, blue_array_is_direct ? blue_array : Buffers.getArray(blue_array), blue_array_is_direct ? Buffers.getDirectBufferByteOffset(blue_array) : Buffers.getIndirectBufferByteOffset(blue_array), blue_array_is_direct);
  }

  /** Entry point to C language function: <code>Bool XF86VidModeSetGammaRamp(Display *  display, int screen, int size, unsigned short *  red_array, unsigned short *  green_array, unsigned short *  blue_array)</code><br>
      @param red_array a direct or array-backed {@link java.nio.ShortBuffer}
      @param green_array a direct or array-backed {@link java.nio.ShortBuffer}
      @param blue_array a direct or array-backed {@link java.nio.ShortBuffer}   */
  private static native boolean XF86VidModeSetGammaRamp1(long display, int screen, int size, Object red_array, int red_array_byte_offset, boolean red_array_is_direct, Object green_array, int green_array_byte_offset, boolean green_array_is_direct, Object blue_array, int blue_array_byte_offset, boolean blue_array_is_direct);

  /** Interface to C language function: <br> <code>Bool XF86VidModeSetGammaRamp(Display *  display, int screen, int size, unsigned short *  red_array, unsigned short *  green_array, unsigned short *  blue_array)</code><br>   */
  public static boolean XF86VidModeSetGammaRamp(long display, int screen, int size, short[] red_array, int red_array_offset, short[] green_array, int green_array_offset, short[] blue_array, int blue_array_offset)  {

    if(red_array != null && red_array.length <= red_array_offset)
      throw new RuntimeException("array offset argument \"red_array_offset\" (" + red_array_offset + ") equals or exceeds array length (" + red_array.length + ")");
    if(green_array != null && green_array.length <= green_array_offset)
      throw new RuntimeException("array offset argument \"green_array_offset\" (" + green_array_offset + ") equals or exceeds array length (" + green_array.length + ")");
    if(blue_array != null && blue_array.length <= blue_array_offset)
      throw new RuntimeException("array offset argument \"blue_array_offset\" (" + blue_array_offset + ") equals or exceeds array length (" + blue_array.length + ")");
        return XF86VidModeSetGammaRamp1(display, screen, size, red_array, Buffers.SIZEOF_SHORT * red_array_offset, false, green_array, Buffers.SIZEOF_SHORT * green_array_offset, false, blue_array, Buffers.SIZEOF_SHORT * blue_array_offset, false);
  }


  // --- Begin CustomJavaCode .cfg declarations
  
    /** Interface to C language function: <br> <code> XRenderPictFormat *  XRenderFindVisualFormat(Display *  dpy, const Visual *  visual); </code>    */
    public static boolean XRenderFindVisualFormat(long dpy, long visual, XRenderPictFormat dest)  {
      if( dest == null ) {
          throw new RuntimeException("dest is null");
      }
      final ByteBuffer destBuffer = dest.getBuffer();
      if( !Buffers.isDirect(destBuffer) ) {
          throw new RuntimeException("dest buffer is not direct");
      }
      return XRenderFindVisualFormat1(dpy, visual, destBuffer);
    }
    /** Entry point to C language function: <code> XVisualInfo *  XGetVisualInfo(Display * , long, XVisualInfo * , int * ); </code>    */
    private static native boolean XRenderFindVisualFormat1(long dpy, long visual, ByteBuffer xRenderPictFormat);
  
    
    /** Interface to C language function: <br> <code> XVisualInfo *  XGetVisualInfo(Display * , long, XVisualInfo * , int * ); </code>    */
    public static XVisualInfo[] XGetVisualInfo(long arg0, long arg1, XVisualInfo arg2, int[] arg3, int arg3_offset)
    {
      if(arg3 != null && arg3.length <= arg3_offset)
        throw new RuntimeException("array offset argument \"arg3_offset\" (" + arg3_offset + ") equals or exceeds array length (" + arg3.length + ")");
      java.nio.ByteBuffer _res;
      _res = XGetVisualInfo1(arg0, arg1, ((arg2 == null) ? null : arg2.getBuffer()), arg3, Buffers.SIZEOF_INT * arg3_offset);
  
      if (_res == null) return null;
      Buffers.nativeOrder(_res);
      final int count = getFirstElement(arg3, arg3_offset);
      if (count <= 0) return null;
      final int esize = _res.capacity() / count;
      if( esize < XVisualInfo.size() ) {
          throw new RuntimeException("element-size "+_res.capacity()+"/"+count+"="+esize+" < "+XVisualInfo.size());
      }
      XVisualInfo[] _retarray = new XVisualInfo[count];
      for (int i = 0; i < count; i++) {
        _res.position(i * esize); // XVisualInfo.size());
        _res.limit   ((1 + i) * esize); // XVisualInfo.size());
        java.nio.ByteBuffer _tmp = _res.slice();
        _res.position(0);
        _res.limit(_res.capacity());
        _retarray[i] = XVisualInfo.create(_tmp);
      }
      return _retarray;
    }
  
    /** Entry point to C language function: <code> XVisualInfo *  XGetVisualInfo(Display * , long, XVisualInfo * , int * ); </code>    */
    private static native java.nio.ByteBuffer XGetVisualInfo1(long arg0, long arg1, java.nio.ByteBuffer arg2, Object arg3, int arg3_byte_offset);
  
    public static native int GetVisualIDFromWindow(long display, long window);
  
    public static native int DefaultVisualID(long display, int screen);
  
    public static native long CreateWindow(long parent, long display, int screen_index, int visualID, int width, int height, boolean input, boolean visible);
    public static native void DestroyWindow(long display, long window);
    public static native void SetWindowPosSize(long display, long window, int x, int y, int width, int height);
  
    public static Point GetRelativeLocation(long display, int screen_index, long src_win, long dest_win, int src_x, int src_y) {
      return (Point) GetRelativeLocation0(display, screen_index, src_win, dest_win, src_x, src_y);
    }
    private static native Object GetRelativeLocation0(long display, int screen_index, long src_win, long dest_win, int src_x, int src_y);
  
    public static boolean QueryExtension(long display, String extensionName) {
      return QueryExtension0(display, extensionName);
    }
    private static native boolean QueryExtension0(long display, String extensionName);
  
    public static native int XCloseDisplay(long display);
    public static native void XUnlockDisplay(long display);
    public static native void XLockDisplay(long display);
  
 private static int getFirstElement(IntBuffer buf)         { return buf.get(buf.position()); }
 private static int getFirstElement(int[] arr, int offset) { return arr[offset]; }
  // ---- End CustomJavaCode .cfg declarations

} // end of class X11Lib
