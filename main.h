#define MAIN_TASK  10
#define LOWPASS_TASK 11
#define HIGHPASS_TASK 11
#define BANDPASS_TASK 11
#define ISR_TASK 9

#define MAX_FILTER_LEN 19

typedef enum {
   term_in,
   term_out,
   command
} buf_type_t;

typedef enum {
    lowpass,
    highpass,
    bandpass
} filter_type_t;

typedef enum {
   channel_a,
   channel_b
} channel_sel_t;

void main_task(uint_32);
void lowpass_task(uint_32);
void highpass_task(uint_32);
void bandpass_task(uint_32);
void isr_task(uint_32);

void adcISR();
void setupISR();

int apply_filter(filter_type_t type);

void add_sample(int sample);
void set_lowpass_hw(int_32 sample_freq, int_32 cutoff_freq);
void set_lowpass_hw_slow(int cutoff_freq, int sample_freq);
void output_signal(int, channel_sel_t, filter_type_t);

typedef struct   td_struct {
   struct td_struct            _PTR_ TD_NEXT;
   struct td_struct            _PTR_ TD_PREV;
   _mqx_uint                         STATE;
   _task_id                          TASK_ID;
   pointer                           STACK_BASE;
   pointer                           STACK_PTR;
   void                        _PTR_ STACK_LIMIT;
   struct ready_q_struct       _PTR_ MY_QUEUE;
   pointer                           MSG_QUEUE_HEAD;
   pointer                           MSG_QUEUE_TAIL;
   _mqx_uint                         MESSAGES_AVAILABLE;
   pointer                           MESSAGE;
   _mqx_uint                         INFO;
   pointer                           MEMORY_RESOURCE_LIST;
   _mqx_uint                         TASK_ERROR_CODE;
   _task_id                          PARENT;
   _mqx_uint                         TEMPLATE_INDEX;
   pointer                           STDIN_STREAM;
   pointer                           STDOUT_STREAM;
   pointer                           STDERR_STREAM;
   uint_16                           TASK_SR;
   uint_16                           RESERVED1;
   TASK_TEMPLATE_STRUCT_PTR          TASK_TEMPLATE_PTR;
   pointer                           TAD_RESERVED;
   MQX_TICK_STRUCT                   TIMEOUT;
   _mqx_uint                         DISABLED_LEVEL;
   _mqx_uint                         FLAGS;
   MQX_TICK_STRUCT                   TIME_SLICE;
   MQX_TICK_STRUCT                   CURRENT_TIME_SLICE;
   _mqx_uint                         BOOSTED;
   struct ready_q_struct     _PTR_   HOME_QUEUE;
   MQX_TICK_STRUCT                   WATCHDOG_TIME;
   pointer                           PROFILER_CONTEXT_PTR;
   pointer                           RUNTIME_ERROR_CHECK_PTR;
   pointer                           MMU_VIRTUAL_CONTEXT_PTR;
   pointer                           ENVIRONMENT_PTR;
   void                  (_CODE_PTR_ EXIT_HANDLER_PTR)(void);
   void                  (_CODE_PTR_ EXCEPTION_HANDLER_PTR)(_mqx_uint, pointer);
   QUEUE_ELEMENT_STRUCT              TD_LIST_INFO;
   QUEUE_ELEMENT_STRUCT              AUX_QUEUE;
   _mqx_uint                         LWEVENT_BITS;
   pointer                           DSP_CONTEXT_PTR;
   pointer                           FLOAT_CONTEXT_PTR;
   pointer                           CRT_TLS;
   pointer                           TOS_RESERVED;

} TD_STRUCT, _PTR_ TD_STRUCT_PTR;