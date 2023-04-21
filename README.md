# trainlog

![](/trainlog_front.png)
![](/trainlog_back.png)

## Requests/Bahn API

captive portal
curl https://10.101.64.10/en/ -H "Host: wifi.bahn.de" -kv -X POST -d login=true -d accept -d CSRFToken=fuck -H "Cookie: csrf=fuck"

ICE API
https://iceportal.de/api1/rs/status

## License

Copyright Jana Marie Hemsing 2022.

This source describes Open Hardware and is licensed under the CERN-OHL-S v2.

You may redistribute and modify this source and make products using it under the terms of the CERN-OHL-S v2 (https://ohwr.org/cern_ohl_s_v2.txt).

This source is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A PARTICULAR PURPOSE. Please see the CERN-OHL-S v2 for applicable conditions.

Source location: https://github.com/Jana-Marie/trainlog

As per CERN-OHL-S v2 section 4, should You produce hardware based on this source, You must where practicable maintain the Source Location visible on the external case of the Gizmo or other products you make using this source.
