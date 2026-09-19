# Mappa Stradale APC per Android

App Android nativa con mappa OpenStreetMap, ricezione BLE del protocollo `$EVENT`/`$STATUS`, registrazione GPS, esportazione CSV e salvataggio della mappa in PNG.

La mappa usa micro-tratti da 10 m, una lunghezza compatibile con un GPS di circa ±3 m e sufficientemente corta per localizzare una singola buca. `TH` definisce la baseline del manto: fino a 110 la baseline è verde (score 100; include il valore osservato da fermo, circa 103), da 110 a 160 scende gradualmente fino a giallo (80), da 160 a 210 fino ad arancione (55), oltre 210 resta arancione (40–55). Gli eventi peggiorano poi lo score locale: severità sotto 40 = 8 punti, 40–59 = 20, 60–79 = 28, 80–100 = 45, pesati per confidenza. Con `TH` basso il singolo evento pesa di più perché emerge su un manto buono; con `TH` alto il tratto parte già giallo/arancione e l'evento aggiunge meno penalità. Gli eventi successivi nello stesso micro-tratto pesano progressivamente di più (fino a +72%), così la densità fa degradare rapidamente solo la porzione realmente interessata. Score: verde 80–100, giallo 60–79, arancione 40–59, rosso 0–39.

## GPS del telefono

Il filtro è calibrato per un errore tipico di circa ±3 m: rifiuta fix oltre 15 m, non conta spostamenti inferiori alla precisione dichiarata (minimo 3 m), applica un filtro leggero ai punti validi e scarta salti oltre 200 m. Un evento viene geolocalizzato solo con una posizione fresca (massimo 15 s).

## Spazio occupato

L'app non conserva offline le tessere OpenStreetMap: la cache viene disabilitata e cancellata all'avvio di una sessione. In memoria restano solo i punti del percorso e gli eventi rilevati; il PNG creato con **Salva mappa** contiene soltanto la porzione inquadrata al momento del salvataggio.

## Compilazione

Aprire questa cartella con Android Studio e avviare il target `app`, oppure eseguire `gradlew.bat assembleDebug`. Per visualizzare le tessere OpenStreetMap occorre una connessione Internet. Abilitare posizione precisa, Bluetooth e autorizzazioni richieste dall'app.
