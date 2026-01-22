using Grasshopper2.UI;
using Grasshopper2.UI.Icon;
using System;
using System.Reflection;

namespace MyGrasshopper2PlugIn1
{
    public sealed class MyGrasshopper2PlugIn1PluginInfo : Grasshopper2.Framework.Plugin
    {
        static T GetAttribute<T>() where T : Attribute => typeof(MyGrasshopper2PlugIn1PluginInfo).Assembly.GetCustomAttribute<T>();

        public MyGrasshopper2PlugIn1PluginInfo()
          : base(new Guid("899710a4-f4aa-4f5f-9fa4-3a6b33dd3dda"),
                 new Nomen(
                    GetAttribute<AssemblyTitleAttribute>()?.Title,
                    GetAttribute<AssemblyDescriptionAttribute>()?.Description),
                 typeof(MyGrasshopper2PlugIn1PluginInfo).Assembly.GetName().Version)
        {
            Icon = AbstractIcon.FromResource("MyGrasshopper2PlugIn1Plugin", typeof(MyGrasshopper2PlugIn1PluginInfo));
        }

        public override string Author => GetAttribute<AssemblyCompanyAttribute>()?.Company;

        public override sealed IIcon Icon { get; }

        public override sealed string Copyright => GetAttribute<AssemblyCopyrightAttribute>()?.Copyright ?? base.Copyright;

        // public override sealed string Website => "https://mywebsite.example.com";

        // public override sealed string Contact => "myemail@example.com";

        // public override sealed string LicenceAgreement => "license or URL";

    }
}