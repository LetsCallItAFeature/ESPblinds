Automatische Steuerung für Velux-Rolläden mit ESP8266 und IR-Leds

Die Steuerung bedient zwei Arten von Rolläden, Metallrolläden und Stoffrolläden, welche in Gruppen zusammengefasst sind (zwei
Metallrolladengruppen, eine Stoffrolladengruppe). Die Rolläden innerhalb einer Gruppe verhalten sich immer exakt gleich.
Die Steuerung richtet sich nach Uhrzeiten in einer Tabelle, bei den Stoffrolläden kann dies jedoch durch die aktuelle Wetterlage
überschrieben werden, um sie vor Regen zu schützen. Rolläden können auch für einen bestimmten Datumsabschnitt dauerhaft
ausgefahren werden, um starkes Erhitzen des Hauses im Sommer zu verhindern.
Der Esp8266 hostet einen lokalen HTML-Webserver, auf welchem die aktuellen Zustände der Rolladengruppen, sowie die Aus- und
Einfahrzeiten dieser am aktuellen Tag und die Wetterdaten, welche zum außerplanmäßigen Hochfahren der Stoffrolläden führen
können, aufgeführt werden.
Zusätzlich beinhaltet die Software ArduinoOTA, das heißt, der Proßessor kann per Wlan von einem Gerät innerhalb des Netzwerks
mit neuer/geupdateter Software beschrieben werden und muss nicht per USB mit dem Gerät verbunden sein.

Um die Rolläden anzusteuern ist es nötig, die Infrarotfernbedienung dieser zu immitieren. Da das genutzte Protokoll in keiner
IR-Library vorhanden ist, muss eine aufgezeichnete Sendung einer Fernbedienung für jeden Rolladen gesendet werden.
Die Timing-Daten der Sendungen wurden analysiert und konnten gut komprimiert werden, da immer nur zwei verschiedene Zeiten
genutzt werden (kann als Boolean gespeichert werden, statt 16 nur noch 1 Bit pro Zeitangabe), zudem treten die Zeiten immer in
gegensätzlichen Paaren auf, wodurch nur die erste Zeitangabe/das erste Bit für  jedes Paar benötigt wird (z.B. wird aus 1001 10,
halbiert den benötigten Speicherplatz). Die Timing-Daten für "Hoch" und "Runter" unterscheiden sich für jeden Rolladen nur an
zwei oder drei Stellen, wodurch nur die Daten für eine der beiden Sendungen gespeichert werden muss und diese Stellen
gegebenenfalls vor der Sendung geändert werden.
Die Kompression in boolsche Werte hätte gereicht, um Bedenken über mangelnden Speicherplatz zu beseitigen, aber es ist trotzdem
schön zu sehen, wie viel man einspaaren kann.
