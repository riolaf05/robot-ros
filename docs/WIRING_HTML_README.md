# 🌐 Schema Interattivo HTML - Come Usarlo

## Apertura del File

### Metodo 1: Browser Locale (Raccomandato)
```bash
# Da terminale
cd docs
open wiring-diagram.html     # macOS
xdg-open wiring-diagram.html # Linux
start wiring-diagram.html    # Windows

# Oppure
firefox wiring-diagram.html
chrome wiring-diagram.html
```

### Metodo 2: Doppio Click
- Trova il file `docs/wiring-diagram.html`
- Doppio click per aprire nel browser predefinito

### Metodo 3: Drag & Drop
- Trascina il file `wiring-diagram.html` in una finestra del browser

## Caratteristiche Interattive

### 🎨 Schemi Colorati
- **Rosso (#ff6b6b)**: Alimentazione +12V
- **Azzurro (#4ecdc4)**: Alimentazione +5V
- **Nero (#333)**: Ground (GND)
- **Verde chiaro (#95e1d3)**: Segnali PWM
- **Rosa (#f38181)**: Segnali Digitali
- **Viola (#aa96da)**: USB/Seriale

### 🖱️ Hover Effects
- Passa il mouse sopra i componenti per evidenziarli
- I collegamenti si ingrandiscono al passaggio del mouse
- Tooltip mostra il nome del componente

### 📑 Tab Navigation
Clicca sulle tab in alto per navigare tra le sezioni:
1. **Overview**: Vista d'insieme del sistema completo
2. **Arduino → L298N**: Collegamenti motori dettagliati
3. **Arduino → Encoder**: Collegamenti sensori encoder
4. **Alimentazione**: Sistema di alimentazione completo
5. **Raspberry Pi**: Collegamenti USB e mapping porte
6. **Checklist**: Lista di controllo interattiva

### ✅ Checklist Interattiva
- ☑️ Spunta le caselle man mano che completi i collegamenti
- 💾 Lo stato viene salvato automaticamente nel browser (localStorage)
- 🖨️ Pulsante "Stampa Checklist" per stampare solo la checklist
- 🔄 La checklist viene ripristinata alla prossima apertura

### 🖨️ Stampa Ottimizzata
- Usa Ctrl+P (Cmd+P su Mac) per stampare
- Layout ottimizzato per stampa su carta A4
- Tutte le sezioni vengono stampate automaticamente
- Separazione automatica delle pagine

## Vantaggi dello Schema HTML

### ✅ Pro
- 🎨 **Visivamente chiaro**: Colori e forme facilitano la comprensione
- 🖱️ **Interattivo**: Hover effects aiutano a identificare i componenti
- 📋 **Checklist**: Traccia i progressi durante l'assemblaggio
- 🖨️ **Stampabile**: Layout ottimizzato per la stampa
- 💾 **Stato salvato**: Riprendi da dove hai lasciato
- 📱 **Responsive**: Funziona anche su tablet/smartphone

### 📊 Confronto con Altri Formati

| Caratteristica | HTML | ASCII (WIRING_VISUAL.md) | Testo (WIRING.md) |
|---------------|------|--------------------------|-------------------|
| Colori | ✅ Sì | ❌ No | ❌ No |
| Interattivo | ✅ Sì | ❌ No | ❌ No |
| Checklist | ✅ Con salvataggio | ✅ Statica | ❌ No |
| Stampa | ✅ Ottimizzata | ⚠️ Base | ⚠️ Base |
| Offline | ✅ Sì | ✅ Sì | ✅ Sì |
| Dettaglio | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |

## Uso Durante l'Assemblaggio

### 1. Preparazione
1. Apri `wiring-diagram.html` nel browser
2. Tieni il browser aperto su un secondo schermo o tablet
3. Stampa la checklist (opzionale)

### 2. Assemblaggio
1. Segui le tab in ordine (Overview → Arduino→L298N → ecc.)
2. Per ogni collegamento:
   - Identifica i componenti sullo schema (colori)
   - Collega i cavi
   - Spunta la casella nella checklist
3. Il browser salva automaticamente i progressi

### 3. Verifica
1. Vai alla tab "Checklist"
2. Verifica che tutte le caselle siano spuntate
3. Segui le verifiche finali con multimetro

## Troubleshooting

### Il file non si apre
**Problema:** Doppio click non funziona

**Soluzione:**
```bash
# Apri con browser specifico
firefox docs/wiring-diagram.html
```

### Le caselle non si salvano
**Problema:** Checklist non ricorda lo stato

**Soluzione:**
- Verifica che il browser permetta localStorage
- Prova con un browser diverso (Chrome, Firefox)
- Controlla che non sia in modalità privata/incognito

### Gli schemi non sono visibili
**Problema:** SVG non viene renderizzato

**Soluzione:**
- Aggiorna il browser all'ultima versione
- Prova un browser diverso
- Verifica che JavaScript sia abilitato

### Lo zoom non funziona
**Soluzione:**
- Usa lo zoom del browser: Ctrl + (Win/Linux) o Cmd + (Mac)
- Oppure Ctrl+Scroll del mouse

## Integrazione con Workflow

### Prima dell'Assemblaggio
1. Leggi [DEPLOYMENT.md](DEPLOYMENT.md) - Sezione "Configurazione Hardware"
2. Leggi [WIRING_VISUAL.md](WIRING_VISUAL.md) - Per dettagli pin-to-pin
3. Apri `wiring-diagram.html` - Per assemblaggio visuale

### Durante l'Assemblaggio
- Usa `wiring-diagram.html` come riferimento principale
- Tieni aperto [WIRING_VISUAL.md](WIRING_VISUAL.md) per checklist dettagliata
- Consulta [WIRING.md](WIRING.md) per note e troubleshooting

### Dopo l'Assemblaggio
- Stampa la checklist completata per i tuoi archivi
- Tieni il file HTML disponibile per manutenzione futura

## Personalizzazione

Il file HTML può essere modificato per:
- Aggiungere nuovi componenti
- Cambiare colori
- Aggiungere note personalizzate
- Modificare la checklist

Per modifiche, apri il file con un editor di testo e modifica la sezione SVG o HTML.

## Requisiti Tecnici

- ✅ Browser moderno (Chrome, Firefox, Edge, Safari)
- ✅ JavaScript abilitato
- ✅ localStorage abilitato (per salvataggio checklist)
- ❌ NON richiede connessione internet
- ❌ NON richiede installazione

## Link Utili

- [WIRING.md](WIRING.md) - Guida testuale completa
- [WIRING_VISUAL.md](WIRING_VISUAL.md) - Schemi ASCII dettagliati
- [DEPLOYMENT.md](DEPLOYMENT.md) - Guida deploy completa
- [README.md](README.md) - Indice documentazione

---

**Suggerimento:** Per la migliore esperienza, usa uno schermo grande o tablet durante l'assemblaggio. Lo schema HTML è ottimizzato per visualizzazione su schermi da almeno 10 pollici.

**Buon assemblaggio! 🤖🔧**
