# PSO Project - Análise de Código Não Utilizado

**Data da Análise**: 2025-12-12  
**Versão do Código**: Edge-Period Method  
**Arquivos Analisados**: 13 arquivos principais (.c)

---

## 📊 Resumo Executivo

| Categoria | Itens Não Usados | Ação Recomendada |
|-----------|------------------|------------------|
| **ISR Handlers** | 0 | ✅ Todos registrados corretamente |
| **API Públicas** | 17 funções | ⚠️ Manter (podem ser usadas externamente) |
| **Funções FIFO** | 5 funções | 🔧 Avaliar remoção (utilidades não usadas) |
| **Funções LED** | 14 funções | 🔧 Avaliar remoção (cores não usadas) |
| **Init/Config** | 5 funções | ⚠️ Avaliar (podem ser legado) |
| **PWM Antigas** | 3 funções | ❌ Remover (depreciadas) |
| **Debug** | 1 função | ⚠️ Manter (uso futuro) |
| **Variáveis Globais** | 6 variáveis | 🔧 Remover (método antigo) |

**Total Estimado**: ~50 funções + 6 variáveis não utilizadas

---

## 📍 CATEGORIA 1: ISR Handlers

**Status**: ✅ **Todos OK** - Registrados no vetor de interrupção

Estas funções são usadas indiretamente pelo hardware/NVIC:

```c
✓ ADC0SS1IntHandler()      // ADC0 Sample Sequencer 1
✓ ADC1SS1IntHandler()      // ADC1 Sample Sequencer 1  
✓ Timer0AIntHandler()      // Timer0A (ADC trigger)
✓ Timer3AIntHandler()      // Timer3A (timeout detection)
✓ UART0IntHandler()        // UART0 RX
✓ WTimer1AIntHandler()     // WTimer1A (RPM edge capture)
✓ WTimer1BIntHandler()     // WTimer1B (reservado)
✓ WTimer5AIntHandler()     // WTimer5A (reservado)
✓ WTimer5BIntHandler()     // WTimer5B (reservado)
✓ SysTick_Handler()        // SysTick (1ms tick)
```

**Recomendação**: ✅ **Manter todas**

---

## 📍 CATEGORIA 2: API Públicas (Não Usadas Internamente)

**Status**: ⚠️ **Manter** - Podem ser usadas por código externo ou futuro

### Funções RPM (pso_rpm.c)

```c
❌ rpm_is_ready()              // Verifica se novo RPM disponível
❌ rpm_clear_ready_flag()      // Limpa flag de dados prontos
❌ rpm_from_period_us()        // Calcula RPM a partir do período
❌ rpm_get_raw_count()         // Retorna contador bruto do timer
❌ rpm_calculate()             // Fórmula antiga edge-count
❌ rpm_is_valid()              // Valida range do RPM
❌ rpm_reset()                 // Reset do sistema RPM
❌ rpm_from_frequency()        // Converte frequência → RPM
❌ rpm_to_frequency()          // Converte RPM → frequência
❌ rpm_is_stopped()            // Verifica se motor parou
```

**Usadas**:
```c
✓ rpm_get_value()              // Usada 1x - em main.c
✓ rpm_get_edge_interval_us()   // Usada 1x - em main.c
✓ rpm_get_filtered()           // Usada 1x - em main.c
```

**Análise**:
- Estas funções formam a **API pública** do módulo RPM
- Podem ser usadas por:
  - Código de teste/calibração
  - Interface com usuário
  - Futuros módulos de análise
  - Aplicações externas via biblioteca

**Recomendação**: ⚠️ **MANTER TODAS**
- São parte da interface pública do módulo
- Baixo custo de manutenção (apenas declarações)
- Podem ser úteis para testes e debug

---

### Funções Timing (pso_timing.c)

```c
❌ timing_enable()             // Habilita/desabilita timing
❌ timing_get_elapsed_ms()     // Tempo decorrido desde início
❌ timing_get_execution_count() // Contador de execuções
❌ timing_get_actual_rate_hz()  // Taxa real de execução
```

**Usadas**:
```c
✓ get_system_time_us()         // Usada 1x - em WTimer1A ISR
```

**Recomendação**: ⚠️ **MANTER**
- Úteis para profiling e otimização
- Monitoramento de performance
- Debug de timing issues

---

### Funções PWM (pso_pwm.c)

```c
❌ pwm_get_current_throttle()  // Retorna throttle atual
❌ pwm_profile_is_running()    // Verifica se perfil está rodando
❌ pwm_profile_execute()       // Executa perfil (wrapper)
```

**Recomendação**: ⚠️ **MANTER**
- API para controle externo
- Monitoramento de estado

---

## 📍 CATEGORIA 3: Funções FIFO Utilitárias

**Status**: 🔧 **Avaliar Remoção** - Não usadas no código atual

```c
❌ fifo_peek()                 // Lê sem remover
❌ fifo_clear()                // Limpa FIFO
❌ fifo_print()                // Imprime conteúdo (debug)
❌ fifo_transfer()             // Transfere entre FIFOs
❌ fifo_available_space()      // Espaço disponível
```

**Usadas**:
```c
✓ fifo_put()                   // Inserir elemento
✓ fifo_get()                   // Remover elemento
✓ fifo_del()                   // Deletar elemento
✓ fifo_is_empty()              // Verificar vazio
✓ fifo_is_full()               // Verificar cheio
✓ fifo_init()                  // Inicializar
✓ fifo_count_elements()        // Contar elementos (usada 2x)
```

**Análise**:
- `fifo_peek()`: Útil para preview sem consumir
- `fifo_clear()`: Útil para reset rápido
- `fifo_print()`: Debug only - pode manter
- `fifo_transfer()`: Ping-pong buffering não implementado
- `fifo_available_space()`: Útil para flow control

**Recomendação**: 
- 🟢 **MANTER**: `fifo_peek()`, `fifo_clear()`, `fifo_available_space()`
- 🟡 **AVALIAR**: `fifo_print()` (somente se debug estiver habilitado)
- 🔴 **REMOVER**: `fifo_transfer()` (ping-pong não usado)

**Economia de Flash**: ~200 bytes se remover todas

---

## 📍 CATEGORIA 4: Funções LED

**Status**: 🔧 **Avaliar Remoção** - Cores/modos não usados

### Funções Não Usadas (14 funções)

```c
// LEDs individuais não usados
❌ led_blue_on()               // Azul ON (usa toggle)
❌ led_blue_off()              // Azul OFF
❌ led_green_off()             // Verde OFF (usa toggle)  
❌ led_red_toggle()            // Vermelho toggle

// Cores compostas não usadas
❌ led_cyan_on()               // Ciano (Azul + Verde)
❌ led_cyan_off()
❌ led_cyan_toggle()
❌ led_purple_on()             // Roxo (Vermelho + Azul)
❌ led_purple_off()
❌ led_purple_toggle()
❌ led_yellow_on()             // Amarelo (Vermelho + Verde)
❌ led_yellow_off()
❌ led_yellow_toggle()
❌ led_white_toggle()          // Branco (todas)
```

### Funções Usadas

```c
✓ led_red_on()                 // Erro
✓ led_red_off()
✓ led_green_on()               // Streaming
✓ led_green_toggle()
✓ led_blue_toggle()            // Standby
✓ led_white_on()               // Finish
✓ led_white_off()
✓ led_all_off()                // Init
```

**Análise**:
- Sistema usa apenas: Vermelho, Verde, Azul, Branco
- Cores compostas (Ciano, Roxo, Amarelo) não são usadas
- Alguns estados intermediários não são usados

**Recomendação**: 
- 🔴 **REMOVER**: Todas as cores compostas (6 funções × 3 = 18 funções)
  - `led_cyan_*()` (3 funções)
  - `led_purple_*()` (3 funções)  
  - `led_yellow_*()` (3 funções)
- 🟡 **AVALIAR**: `led_blue_on/off()` - só usa toggle
- 🟢 **MANTER**: Cores básicas usadas

**Economia de Flash**: ~300-400 bytes

---

## 📍 CATEGORIA 5: Funções de Inicialização/Configuração

**Status**: ⚠️ **Avaliar** - Podem ser código legado

```c
❌ myISR_Config()              // Placeholder vazio (pso_init.c)
❌ myPWM_Init()                // PWM init legado (não usada)
❌ myUART2Config_Init()        // UART2 config (9600 baud)
❌ configure_systick()         // SysTick config manual
❌ pso_spi2_config()           // SPI2 para SD card
```

**Análise por Função**:

### `myISR_Config()`
- **Localização**: pso_init.c linha 58
- **Conteúdo**: Função vazia (placeholder)
- **Recomendação**: 🔴 **REMOVER** (não faz nada)

### `myPWM_Init()`
- **Problema**: Existe `pso_pwm_config()` que é usada
- **Recomendação**: 🔴 **REMOVER** (depreciada)

### `myUART2Config_Init()`
- **Função**: Configura UART2 (PD6/PD7) a 9600 baud
- **Uso**: Debug/comunicação alternativa
- **Recomendação**: 🟡 **AVALIAR** 
  - Se UART2 não é usada → remover
  - Se é necessária → adicionar chamada em system_init()

### `configure_systick()`
- **Função**: Configuração manual do SysTick
- **Atual**: `timing_init()` configura o SysTick
- **Recomendação**: 🔴 **REMOVER** (substituída por timing_init)

### `pso_spi2_config()`
- **Função**: Configuração do SPI2 para SD card
- **Status**: SD card não implementado completamente
- **Recomendação**: 🟡 **MANTER** (feature futura)

**Economia de Flash**: ~150-200 bytes

---

## 📍 CATEGORIA 6: Funções PWM Antigas/Depreciadas

**Status**: ❌ **REMOVER** - Substituídas por novo sistema

```c
❌ fun_linear()                // PWM linear (antigo)
❌ fun_trapezoid()             // PWM trapézio (antigo)
❌ decrement()                 // Decrementa PWM
```

**Usadas**:
```c
✓ increment()                  // Usada em Timer3A ISR
```

**Análise**:
- Sistema antigo de PWM foi substituído por:
  - `execute_linear_profile()`
  - `execute_trapezoid_profile()`
  - `execute_step_profile()`
- `decrement()` não é usada (só `increment()`)

**Recomendação**: 🔴 **REMOVER TODAS AS 3**
- Código depreciado
- Substituído por sistema mais robusto
- Não há referências no código atual

**Economia de Flash**: ~300-500 bytes

---

## 📍 CATEGORIA 7: Funções de Debug

**Status**: ⚠️ **Manter** - Úteis para desenvolvimento

```c
❌ debug_timing_pulse()        // Gera pulso para osciloscópio
```

**Usadas**:
```c
✓ debug_gpio_init()            // Inicializa pinos debug
✓ debug_timing_measure()       // Mede tempo de execução
```

**Macros Usadas**:
```c
✓ DEBUG_ADC_TOGGLE()           // Usada em main.c
✓ DEBUG_STATE_TOGGLE()         // Comentada mas disponível
```

**Análise**:
- `debug_timing_pulse()` é complementar a `debug_timing_measure()`
- Útil para gerar marcadores em osciloscópio
- Baixo custo (~50 bytes)

**Recomendação**: 🟢 **MANTER**
- Ferramenta de debug valiosa
- Pode ser necessária para otimização futura

---

## 📍 CATEGORIA 8: Variáveis Globais Não Usadas

**Status**: 🔧 **Remover** - Legado do método antigo

```c
❌ g_system_ms_counter         // Contador MS (depreciado)
❌ g_timer_a3_scan_flag        // Flag Timer3A (método edge-count)
❌ profile_complete            // Flag perfil completo
❌ sample_counter              // Contador de amostras  
❌ scan_period_actual          // Período de scan
❌ wt1cpp0_tav_buffer          // Buffer WTimer (método antigo)
```

### Análise Detalhada

#### `g_system_ms_counter`
- **Localização**: pso_timing.c
- **Tipo**: `volatile uint32_t`
- **Problema**: Substituída por `g_system_tick_counter`
- **Recomendação**: 🔴 **REMOVER**

#### `g_timer_a3_scan_flag`
- **Localização**: pso_isr.c
- **Tipo**: `volatile uint32_t`
- **Uso Original**: Flag do Timer3A no método edge-count
- **Problema**: Edge-period não usa esta flag
- **Recomendação**: 🔴 **REMOVER**

#### `profile_complete`
- **Localização**: main.c linha 89
- **Tipo**: `uint8_t`
- **Uso**: Flag de perfil PWM completo
- **Problema**: Setada mas nunca lida
- **Recomendação**: 🔴 **REMOVER** ou 🟡 **USAR** para feedback

#### `sample_counter`
- **Localização**: pso_timing.c
- **Tipo**: `uint32_t`
- **Problema**: Declarada mas nunca usada
- **Recomendação**: 🔴 **REMOVER**

#### `scan_period_actual`
- **Localização**: main.c linha 87
- **Tipo**: `uint16_t`
- **Problema**: Nunca inicializada nem lida
- **Recomendação**: 🔴 **REMOVER**

#### `wt1cpp0_tav_buffer`
- **Localização**: pso_rpm.c (declarada) e pso_isr.c (usada no método antigo)
- **Tipo**: `uint32_t`
- **Uso Original**: Buffer para contagem de pulsos (edge-count)
- **Problema**: Método edge-period não usa contagem
- **Recomendação**: 🔴 **REMOVER**

**Economia de RAM**: ~24 bytes

---

## 📍 CATEGORIA 9: Funções de Dados (pso_data.c)

**Status**: ⚠️ **Código de Teste** - Avaliar necessidade

```c
❌ copy_raw_data()             // Copia buffer UART
❌ read_raw_data()             // Popula buffer com "PSO-v1"
```

**Análise**:
- Estas funções parecem ser código de teste
- `read_raw_data()` escreve string fixa "PSO-v1\r\n"
- Não são usadas no fluxo principal
- Sistema atual usa `packet_data()` do ulink.c

**Recomendação**: 🔴 **REMOVER**
- Código de teste/exemplo
- Não faz parte do sistema de produção

**Economia de Flash**: ~100 bytes

---

## 📍 CATEGORIA 10: Funções ULINK (ulink.c)

**Status**: ⚠️ **Avaliar** - Protocolo alternativo

```c
❌ uart_write()                // Cria mensagem ULINK
❌ uart_read()                 // Lê mensagem ULINK
```

**Usado**:
```c
✓ packet_data()                // Prepara pacote de dados
✓ copy_data()                  // Copia dados para buffer
✓ create_message()             // Cria mensagem completa
✓ parse_message()              // Parser de mensagens
✓ create_checksum()            // CRC
✓ accumulate_checksum()        // CRC acumulativo
```

**Análise**:
- `uart_write()` e `uart_read()` não são usadas
- Sistema atual usa `copy_data()` + transmissão direta
- Podem ser wrappers para uso futuro

**Recomendação**: 🟡 **AVALIAR**
- Se forem necessárias para compatibilidade → manter
- Se não forem usadas → remover

---

## 🎯 RECOMENDAÇÕES FINAIS

### ✅ Ações Imediatas (Remoção Segura)

#### 1. Remover Funções Depreciadas (Alta Prioridade)
```c
// pso_pwm.c
- fun_linear()
- fun_trapezoid()  
- decrement()
```
**Economia**: ~500 bytes

#### 2. Remover Variáveis do Método Antigo (Alta Prioridade)
```c
// pso_isr.c / pso_rpm.c
- g_timer_a3_scan_flag
- wt1cpp0_tav_buffer

// main.c
- scan_period_actual
- profile_complete (se não for usada)

// pso_timing.c
- g_system_ms_counter
- sample_counter
```
**Economia**: ~24 bytes RAM

#### 3. Remover Funções Init Vazias (Alta Prioridade)
```c
// pso_init.c
- myISR_Config()        // Função vazia
- configure_systick()   // Substituída por timing_init()
```
**Economia**: ~100 bytes

#### 4. Remover Funções de Teste (Média Prioridade)
```c
// pso_data.c
- copy_raw_data()
- read_raw_data()
```
**Economia**: ~100 bytes

#### 5. Remover LEDs Não Usados (Média Prioridade)
```c
// pso_led.c - Cores compostas não usadas
- led_cyan_on/off/toggle()      (3 funções)
- led_purple_on/off/toggle()    (3 funções)
- led_yellow_on/off/toggle()    (3 funções)
```
**Economia**: ~300-400 bytes

### ⚠️ Avaliar Antes de Remover

#### 1. UART2 Configuration
```c
// pso_init.c
- myUART2Config_Init()
```
**Decisão**: Verificar se UART2 (PD6/PD7) é necessária

#### 2. SPI2 Configuration  
```c
// pso_init.c
- pso_spi2_config()
```
**Decisão**: Manter se SD card for implementado

#### 3. FIFO Transfer
```c
// fifo.c
- fifo_transfer()
```
**Decisão**: Remover se ping-pong buffering não for usado

#### 4. ULINK Wrappers
```c
// ulink.c
- uart_write()
- uart_read()
```
**Decisão**: Verificar se são necessários para compatibilidade

### 🟢 Manter (API Pública)

**Não remover** - fazem parte da interface pública:

```c
// pso_rpm.h - API RPM
✓ Todas as funções rpm_*()

// pso_timing.h - API Timing
✓ Todas as funções timing_*()

// pso_pwm.h - API PWM
✓ Todas as funções pwm_*()

// fifo.h - Utilitários essenciais
✓ fifo_peek(), fifo_clear(), fifo_available_space()

// pso_debug.h - Ferramentas de debug
✓ debug_timing_pulse()
```

---

## 📊 Estimativa de Economia Total

| Categoria | Flash (bytes) | RAM (bytes) |
|-----------|---------------|-------------|
| PWM Depreciado | 500 | 0 |
| LEDs Não Usados | 350 | 0 |
| Init/Config | 200 | 0 |
| Data Test | 100 | 0 |
| Variáveis Antigas | 0 | 24 |
| **TOTAL ESTIMADO** | **~1150** | **24** |

**Percentual**: ~0.4% do Flash (256KB) e ~0.07% da RAM (32KB)

---

## 📝 Checklist de Refatoração

### Fase 1: Remoção Segura (Sem Riscos)
- [ ] Remover `fun_linear()`, `fun_trapezoid()`, `decrement()`
- [ ] Remover `myISR_Config()` (função vazia)
- [ ] Remover `copy_raw_data()`, `read_raw_data()`
- [ ] Remover `g_timer_a3_scan_flag`
- [ ] Remover `scan_period_actual`
- [ ] Remover `g_system_ms_counter`
- [ ] Remover `sample_counter`

### Fase 2: Avaliação (Requer Decisão)
- [ ] Avaliar necessidade do UART2 → decidir sobre `myUART2Config_Init()`
- [ ] Avaliar uso de `profile_complete` → remover ou implementar uso
- [ ] Avaliar `fifo_transfer()` → remover se ping-pong não usado
- [ ] Avaliar `wt1cpp0_tav_buffer` → confirmar não uso no novo método

### Fase 3: Otimização Opcional (Baixa Prioridade)
- [ ] Remover LEDs de cores compostas (cyan, purple, yellow)
- [ ] Considerar remoção de `configure_systick()`
- [ ] Avaliar `uart_write()` e `uart_read()` do ulink.c

### Fase 4: Testes Após Refatoração
- [ ] Compilar e verificar warnings
- [ ] Testar funcionamento completo do sistema
- [ ] Verificar redução do tamanho do binário
- [ ] Validar que nenhuma funcionalidade foi quebrada

---

## 🔍 Comandos para Verificação

### Buscar referências antes de remover:
```bash
# Exemplo: verificar se função é usada
grep -r "nome_da_funcao" *.c *.h

# Verificar declarações extern
grep -r "extern.*nome_da_variavel" *.c *.h

# Listar símbolos no binário
arm-none-eabi-nm PSO.out | grep nome_da_funcao
```

### Comparar tamanho do binário:
```bash
# Antes da refatoração
arm-none-eabi-size PSO.out

# Depois da refatoração  
arm-none-eabi-size PSO_refactored.out
```

---

## ⚠️ Avisos Importantes

1. **Não remover ISR Handlers**: Mesmo que pareçam não usados, são registrados no vetor de interrupção

2. **API Pública**: Funções exportadas (rpm_*, timing_*, pwm_*) devem ser mantidas mesmo que não usadas internamente

3. **Código de Debug**: Funções debug_* são úteis para desenvolvimento futuro

4. **Backup**: Fazer backup do código antes de remover funções

5. **Testes**: Testar completamente após cada fase de remoção

---

**Gerado por**: Análise automatizada de código  
**Próxima Revisão**: Após implementação das mudanças
