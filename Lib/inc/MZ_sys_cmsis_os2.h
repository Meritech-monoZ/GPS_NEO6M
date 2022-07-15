/**
 * @file MZ_sys_cmsis_os2.h
 * @author SKM
 * @date Mar 16 2021 
 * @brief Monoz System CMSIS OS2
 * This is an abstraction layer and includes FreeRTOS configuration, here can create the App and can also
 * migrate to different versions with user required changes.
 */


#include "cmsis_os2.h"
#include "MZ_common.h"
#include "FreeRTOSConfig.h"
#include "projdefs.h"
#include "portmacro.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "MZ_error_handler.h"


#ifndef MZ_SYS_CMSIS_OS2_H_
#define MZ_SYS_CMSIS_OS2_H_									///< Defines Monoz System CMSIS OS2
#ifdef __cplusplus
extern "C" {
#endif
															
#define CMSISOS2_INFINITE32		(0xFFFFFFFF)				///< Defines address of CMISOS_INFINITE32
															
#define SEMA_TIMEOUT			(CMSISOS2_INFINITE32)		///< Defines SEMA_TIMEOUT
															
#define MAILBOX_TIMEOUT			(CMSISOS2_INFINITE32)		///< Defines MAILBOX_TIMEOUT
															
#define DEFAULT_STACK_SIZE		(256)						///< Defines size of the DEFAULT STACK


/** @brief  \a mz_mutex_t variable of type \a osMutexId_t */
typedef osMutexId_t 			mz_mutex_t;					
/** @brief \a mz_semaphore_t variable of type \a osSemaphoreId_t */      	
typedef osSemaphoreId_t			mz_semaphore_t;				
/** @brief \a mz_mailbox_t variable of type \a osMessageQueueId_t */		
typedef osMessageQueueId_t 		mz_mailbox_t;				
/** @brief \a mz_thread_t variable of type \a osThreadId_t */				
typedef osThreadId_t			mz_thread_t;				
/** @brief \a mz_thread_prio_t variable of type \a osPriority_t*/			
typedef osPriority_t			mz_thread_prio_t;			
/** @brief \a mz_timer_t variable of type \a TimerHandle_t */				
typedef TimerHandle_t			mz_timer_t;					
/** @brief \a mz_timer_fp variable of type \a TimerCallbackFunction_t */	
typedef TimerCallbackFunction_t mz_timer_fp;


/** 
 * @defgroup SYSTEM SYSTEM
 * Status of System
 * @{
 * @fn uint32_t mz_sys_now(void)
 * @brief Get the RTOS kernel system timer count.
 * @return RTOS kernel current system timer count as 32-bit value.
 */ 
uint32_t mz_sys_now(void);
/**@} */

// CMSIS V2 mutex related Abstraction
/** @defgroup MUTEX MUTEX
 * CMSIS_V2_MUTEX_RELATED_ABSTRACTION CMSIS_V2_MUTEX_RELATED_ABSTRACTION
 * @{
 * @fn uint8_t mz_mutex_create(mz_mutex_t * c)
 * @brief CMSIS V2 Mutex Related Abstarction
 * 
 * Create and Initialize a Mutex object.
 * @param c  mz_mutex_t
 * @return mutex ID for reference by other functions or NULL in case of error.
 */  
uint8_t mz_mutex_create(mz_mutex_t * c);

/**
 * @fn uint8_t mz_mutex_lock(mz_mutex_t * c)
 * @brief CMSIS V2 Mutex Related Abstarction
 * 
 * Acquire a Mutex or timeout if it is locked
 * @param c mz_mutex_t
 * @return status code that indicates the execution status of the function.
 */
uint8_t mz_mutex_lock(mz_mutex_t * c);

/**
 * @fn uint8_t mz_mutex_unlock(mz_mutex_t * c)
 * @brief CMSIS V2 Mutex Related Abstarction
 * 
 * Release a Mutex that was acquired by
 * @param c mz_mutex_t
 * @return status code that indicates the execution status of the function.
 */
uint8_t mz_mutex_unlock(mz_mutex_t * c);

/**
 * @fn uint8_t mz_mutex_delete(mz_mutex_t * c)
 * @brief CMSIS V2 Mutex Related Abstarction
 * 
 * Delete a Mutex object
 * @param c mz_mutex_t
 * @return status code that indicates the execution status of the function.
 */
uint8_t mz_mutex_delete(mz_mutex_t * c);

/**
 * @fn uint8_t mz_mutex_isvalid(mz_mutex_t * c)
 * @brief CMSIS V2 Mutex Related Abstarction
 * 
 * Check the Mutex Object Validation
 * @param c mz_mutex_t
 * @return status code that indicates the execution status of the function.
 */
uint8_t mz_mutex_isvalid(mz_mutex_t * c);

/**
 * @fn uint8_t mz_mutex_invalid(mz_mutex_t* c)
 * @brief CMSIS V2 Mutex Related Abstarction
 * 
 * Mutex Object created is invalid
 * @param c mz_mutex_t
 * @return 1
 */
uint8_t mz_mutex_invalid(mz_mutex_t* c);
/** @} */

// CMSIS V2 semaphore related Abstraction
/**
 * @defgroup SEMAPHORE SEMAPHORE
 * CMSIS_V2_SEMAPHORE_RELATED_ABSTRACTION CMSIS_V2_SEMAPHORE_RELATED_ABSTRACTION
 * @{
 * @fn uint8_t mz_sem_create(mz_semaphore_t * c, uint8_t cnt, uint8_t init)
 * @brief CMSIS V2 Semaphore Related Abstarction
 * 
 * Create and Initialize a Semaphore object
 * @param c mz_mutex_t
 * @param cnt uint8_t
 * @param init uint8_t
 * @return semaphore ID for reference by other functions or NULL in case of error.
 */
uint8_t mz_sem_create(mz_semaphore_t * c, uint8_t cnt, uint8_t init);

/**
 * @fn uint8_t mz_sem_delete(mz_semaphore_t * c)
 * @brief CMSIS V2 Semaphore Related Abstarction
 * 
 * Delete a Semaphore object.
 * @param c mz_semaphore_t
 * @return status code that indicates the execution status of the function.
 */
uint8_t mz_sem_delete(mz_semaphore_t * c);

/**
 * @fn uint32_t mz_sem_wait(mz_semaphore_t * c, uint32_t timeout)
 * @brief CMSIS V2 Semaphore Related Abstarction
 * 
 * Acquire a Semaphore token or timeout if no tokens are available.
 * @param c mz_semaphore_t
 * @param timeout uint32_t
 * @return status code that indicates the execution status of the function.
 */
uint32_t mz_sem_wait(mz_semaphore_t * c, uint32_t timeout);

/**
 * @fn uint8_t mz_sem_release(mz_semaphore_t * c)
 * @brief CMSIS V2 Semaphore Related Abstarction
 * 
 * Release a Semaphore token up to the initial maximum count.
 * @param c mz_semaphore_t
 * @return status code that indicates the execution status of the function.
 */
uint8_t mz_sem_release(mz_semaphore_t * c);

/**
 * @fn uint8_t mz_sem_isvalid(mz_semaphore_t * c)
 * @brief CMSIS V2 Semaphore Related Abstarction
 * 
 * Check the Semaphore object validation.
 * @param c mz_semaphore_t
 * @return status code that indicates the execution status of the function.
 */
uint8_t mz_sem_isvalid(mz_semaphore_t * c);

/**
 * @fn uint8_t mz_sem_invalid(mz_semaphore_t * c)
 * @brief CMSIS V2 Semaphore Related Abstarction
 * 
 * Semaphore object is invalid.
 * @param c mz_semaphore_t
 * @return status code that indicates the execution status of the function.
 */
uint8_t mz_sem_invalid(mz_semaphore_t * c);
/** @} */

// CMSIS V2 message queue related Abstraction
/**
 * @defgroup MAILBOX MAILBOX
 * CMSIS_V2_MESSAGE_QUEUE_RELATED_ABSTRACTION CMSIS_V2_MESSAGE_QUEUE_RELATED_ABSTRACTION
 * @{
 * @fn uint8_t mz_mailbox_create(mz_mailbox_t * c, size_t no_of_mail, size_t mail_size)
 * @brief CMSIS V2 Message Queue Related Abstarction
 * 
 * Create and Initialize a Message Queue object.
 * @param c mz_mailbox_t
 * @param no_of_mail  size_t
 * @param mail_size   size_t
 * @return message queue ID for reference by other functions or NULL in case of error.
 */
uint8_t mz_mailbox_create(mz_mailbox_t * c, size_t no_of_mail, size_t mail_size);

/**
 * @fn uint8_t mz_mailbox_delete(mz_mailbox_t * c)
 * @brief CMSIS V2 Message Queue Related Abstarction
 * 
 * Delete Message Queue object.
 * @param c mz_mailbox_t
 * @return message queue ID for reference by other functions or NULL in case of error.
 */
uint8_t mz_mailbox_delete(mz_mailbox_t * c);

/**
 * @fn mz_error_t mz_mailbox_put(mz_mailbox_t * c, void* m)
 * @brief CMSIS V2 Message Queue Related Abstarction
 * 
 * Put a Message into a Queue or timeout if Queue is full. \a `osWaitForever' is set
 * @param c mz_mailbox_t
 * @param m void
 * @return status code that indicates the execution status of the function.
 * @note Error handler for Message Queue Related Abstarction
 */
mz_error_t mz_mailbox_put(mz_mailbox_t * c, void* m);

/**
 * @fn mz_error_t mz_mailbox_get(mz_mailbox_t * c, void* m, uint32_t timeout)
 * @brief CMSIS V2 Message Queue Related Abstarction
 * 
 * Get a Message from a Queue or timeout if Queue is empty.
 * @param c mz_mailbox_t
 * @param m void
 * @param timeout uint32_t
 * @return status code that indicates the execution status of the function.
 * @note Error handler for Message Queue Related Abstarction
 */
mz_error_t mz_mailbox_get(mz_mailbox_t * c, void* m, uint32_t timeout);

/**
 * 
 * @fn mz_error_t mz_mailbox_putnow(mz_mailbox_t * c, void* m)
 * @brief CMSIS V2 Message Queue Related Abstarction
 * 
 * Put a Message into a Queue or timeout if Queue is full. \a `timeout' is set to 0.
 * @param c mz_mailbox_t
 * @param m void
 * @return status code that indicates the execution status of the function.
 * @note Error handler for Message Queue Related Abstarction
 */
mz_error_t mz_mailbox_putnow(mz_mailbox_t * c, void* m);

/**
 * @fn mz_error_t mz_mailbox_getnow(mz_mailbox_t * c, void* m)
 * @brief CMSIS V2 Message Queue Related Abstarction
 * 
 * Get a Message from a Queue or timeout if Queue is empty.
 * @param c mz_mailbox_t
 * @param m void
 * @return status code that indicates the execution status of the function.
 * @note Error handler for Message Queue Related Abstarction
 */
mz_error_t mz_mailbox_getnow(mz_mailbox_t * c, void* m);

/**
 * @fn uint8_t mz_mailbox_isvalid(mz_mailbox_t * c)
 * @brief CMSIS V2 Message Queue Related Abstarction Error Scenerio
 * 
 * Check the Message Queue object validation.
 * @param c mz_mailbox_t
 * @return status code that indicates the execution status of the function.
 */
uint8_t mz_mailbox_isvalid(mz_mailbox_t * c);

/**
 * @fn uint8_t mz_mailbox_invalid(mz_mailbox_t * c)
 * @brief CMSIS V2 Message Queue Related Abstarction
 * 
 * Message Queue object is invalid.
 * @param c mz_mailbox_t
 * @return 1
 */
uint8_t mz_mailbox_invalid(mz_mailbox_t * c);
/** @} */

// CMSIS V2 thread related Abstraction
/** @defgroup THREAD THREAD
 * CMSIS_V2_THREAD_RELATED_ABSTRACTION CMSIS_V2_THREAD_RELATED_ABSTRACTION
 * @{
 * @fn uint8_t mz_thread_create(mz_thread_t* t,const char* name,mz_fn thread_func,void* const arg,mz_thread_prio_t prio,StackType_t * stack_mem,size_t stack_size,StaticTask_t * cb_mem,uint32_t cb_size)
 * @brief CMSIS V2 Thread Related Abstarction
 * 
 * Create a thread and add it to Active Threads.
 * @param t mz_thread_t 
 * @param name const char
 * @param thread_func mz_fn
 * @param arg void * const
 * @param prio mz_thread_prio_t
 * @param stack_mem StackType_t
 * @param stack_size size_t
 * @param cb_mem StaticTask_t
 * @param cb_size uint32_t
 * @return thread ID for reference by other functions or NULL in case of error.
 */
uint8_t mz_thread_create(mz_thread_t* t,const char* name,mz_fn thread_func,void* const arg,mz_thread_prio_t prio,StackType_t * stack_mem,size_t stack_size,StaticTask_t * cb_mem,uint32_t cb_size);

/**
 * @fn uint8_t mz_thread_terminate(mz_thread_t* t)
 * @brief CMSIS V2 Thread Related Abstarction
 * 
 * Terminate the execution of thread
 * @param t mz_thread_t
 * @return 1
 */
uint8_t mz_thread_terminate(mz_thread_t* t);

/**
 * @fn uint8_t mz_thread_yield(void)
 * @brief CMSIS V2 Thread Related Abstarction
 * 
 * Pass control to next thread that is in state READY.
 * @return 1
 */
uint8_t mz_thread_yield(void);
/** @} */

/**
 * @defgroup MEMORY_MANAGEMENT MEMORY MANAGEMENT
 * Memory Allocation
 * @{
 * @fn void * mz_malloc(size_t size)
 * @brief CMSIS V2 Thread Related Abstarction
 * 
 * Map to the memory management routines required for the port.
 * @param size size_t
 */
void * mz_malloc(size_t size);

/**
 * @fn void mz_free(void * _ptr)
 * @brief CMSIS V2 Thread Related Abstarction
 *
 * Map to the memory management routines required for the port.
 * @param _ptr void
 */
void mz_free(void * _ptr);
/** @} */
#ifdef __cplusplus
}
#endif
#endif /* MZ_SYS_CMSIS_OS2_H_ */
