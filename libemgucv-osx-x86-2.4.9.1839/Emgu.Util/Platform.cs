//----------------------------------------------------------------------------
//  Copyright (C) 2004-2012 by EMGU. All rights reserved.       
//----------------------------------------------------------------------------
using System;
using System.Runtime.InteropServices;
using Emgu.Util.TypeEnum;

namespace Emgu.Util
{
   /// <summary>
   /// Provide information for the platform which is using. 
   /// </summary>
   public static class Platform
   {
      private static readonly OS _os;
      private static readonly Runtime _runtime;

      [DllImport("c")]
      private static extern int uname(IntPtr buffer);

      static Platform()
      {
#if IOS
         _os = OS.IOS;
         _runtime = Runtime.Mono;
#elif ANDROID
         _os = OS.Android;
         _runtime = Runtime.Mono;
#else
         PlatformID pid = Environment.OSVersion.Platform;
         if (pid == PlatformID.MacOSX)
         {
            //This never works, it is a bug in Mono
            _os = OS.MacOSX;
         } else
         {
            int p = (int) pid;
            _os = ((p == 4) || (p == 128)) ? OS.Linux : OS.Windows;

            if (_os == OS.Linux)
            {  //Check if the OS is Mac OSX
               IntPtr buf = IntPtr.Zero;
               try
               {
                  buf = Marshal.AllocHGlobal(8192);
                  // This is a hacktastic way of getting sysname from uname () 
                if (uname(buf) == 0){ 
                    string os = Marshal.PtrToStringAnsi(buf); 
                    if (os == "Darwin") 
                        _os = OS.MacOSX; 
                } 
               } finally
               {
                  if (buf != IntPtr.Zero) Marshal.FreeHGlobal(buf);
               }
            }
         }
         _runtime = (Type.GetType("System.MonoType", false) != null) ? Runtime.Mono : Runtime.DotNet;
#endif
      }

      /// <summary>
      /// Get the type of the current operating system
      /// </summary>
      public static OS OperationSystem
      {
         get
         {
            return _os;
         }
      }

      /// <summary>
      /// Get the type of the current runtime environment
      /// </summary>
      public static Runtime Runtime
      {
         get
         {
            return _runtime;
         }
      }
   }
}
