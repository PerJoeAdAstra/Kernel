/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"

/* Since we *know* there will be 2 processes, stemming from the 2 user
 * programs, we can
Kernel_in_progress/kernel/hilevel.c
 *
 * - allocate a fixed-size process table (of PCBs), and then maintain
 *   an index into it for the currently executing process,
 * - employ a fixed-case of round-robin scheduling: no more processes
 *   can be created, and neither is able to terminate.
 */

pcb_t pcb[ 100 ]; int executing = 0;
int next_pid = 2; int occupied [ 100 ];
pipe_t pipes[ 100 ]; int pipe_occupied[ 100 ];

void scheduler( ctx_t* ctx ) { //swap between console and programs (more responsive)
  memcpy( &pcb[ executing ].ctx, ctx, sizeof( ctx_t ) ); // preserve executing program context
  if( pcb [ executing ].status != STATUS_TERMINATED ){
    pcb[ executing ].status = STATUS_READY;                // update executing program status
  }
  pcb[ executing ].pri_age = 0;                          // Reset executing program priority age
    
  for( int i = 0; i < 100; i++ ){                  // update the priority age of other processes 
    if( pcb[ i ].status == STATUS_READY ){
      if( i != ( executing ) ){
          pcb[ i ].pri_age++;
      }
    }
  }
    
  for(int i = 0; i < 100; i++){                  // find highest priority program
    if( pcb[ i ].status == STATUS_READY ){
      if( pcb[ i ].priority + pcb[ i ].pri_age > pcb[ executing ].priority + pcb[ executing ].pri_age){
          executing = i;
      }
    }
  }
    
  memcpy( ctx, &pcb[ executing ].ctx, sizeof( ctx_t ) );  // restore next program into register
  pcb[ executing ].status = STATUS_EXECUTING;             // update next program status
    
  return;
}

extern void     main_P3();
extern void     main_P4();
extern void     main_console();

extern uint32_t tos_svc;
extern uint32_t tos_PS;

void hilevel_handler_rst( ctx_t* ctx              ) {
  
  /* Initialise PCBs representing processes stemming from execution of
   * the two user programs.  Note in each case that
   *
   * - the CPSR value of 0x50 means the processor is switched into USR
   *   mode, with IRQ interrupts enabled, and
   * - the PC and SP values matche the entry point and top of stack.
   */
    
  pcb[ 0 ].pid      = 1;                    //load console into PCB [ 0 ]
  pcb[ 0 ].status   = STATUS_READY;
  pcb[ 0 ].ctx.cpsr = 0x50;
  pcb[ 0 ].ctx.pc   = ( uint32_t )( &main_console );
  pcb[ 0 ].ctx.sp   = ( uint32_t )( &tos_PS ); 
  pcb[ 0 ].priority = 1;
  occupied[ 0 ] = 1;
    
  /* Once the PCBs are initialised, we select the terminal to be 
   * restored (i.e., executed) when the function then returns.
   */

  memcpy( ctx, &pcb[ 0 ].ctx, sizeof( ctx_t ) );
  pcb[ 0 ].status = STATUS_EXECUTING;
  executing = 0;

  TIMER0->Timer1Load  = 0x00100000; // select period = 2^20 ticks ~= 1 sec
  TIMER0->Timer1Ctrl  = 0x00000002; // select 32-bit   timer
  TIMER0->Timer1Ctrl |= 0x00000040; // select periodic timer
  TIMER0->Timer1Ctrl |= 0x00000020; // enable          timer interrupt
  TIMER0->Timer1Ctrl |= 0x00000080; // enable          timer

  GICC0->PMR          = 0x000000F0; // unmask all            interrupts
  GICD0->ISENABLER1  |= 0x00000010; // enable timer          interrupt
  GICC0->CTLR         = 0x00000001; // enable GIC interface
  GICD0->CTLR         = 0x00000001; // enable GIC distributor

  int_enable_irq();

  return;
}

//translates the pid to a pcb index, returns -1 if pid not found
int pidToIndex( int pid ){ 
  int index = -1;
  for( int i = 0; i < 100; i++ ){
    if( pcb[ i ].pid == pid) {              
      index = i;
      break;
    }
  }
  return index;
}

//returns the index of the next available table slot, returns -1 if table is full
int lowestAvailablePcb(){ 
  int lowest = 0;
  for( int i = 0; i < 100; i++ ){
    if( occupied[ i ] == 0 ){
      return i; 
    }
  }
  return -1;
}

//terminates a process given its id
void terminateProcess( int pid ){
  int index = pidToIndex(pid);
  if( index != -1 ){
    pcb[ index ].status = STATUS_TERMINATED; //set the status of the program to terminated
    occupied[ index ] = 0;                   //set the availiablility to true
  }
  return;
}

//returns the next available option for a array, looping to zero
int next( int start, int max ){
  if( start + 1 == max ) return 0;
  else return (start + 1);
}

//puts a byte in a specified pipe, returns 1 if successful 0 if full or blocked
int write_to_pipe( int i, char x){
  if(pipes[ i ].writeBlock == false){
    if(next(pipes[ i ].end, pipes[ i ].max) > pipes[ i ].start) {  //checks if pipe is blocked 
      pipes[ i ].writeBlock = true;                          //or becomes  blocked
    }
    else{
      *pipes[ i ].data[next(pipes[ i ].end, pipes[ i ].max)] = x;   //write data and 
      pipes[ i ].end = next(pipes[ i ].end, pipes[ i ].max);                       //adjust pointers,
      if( pipes[ i ].readBlock ) pipes[ i ].readBlock = false;                     //unblock read
      return 1;
    }
  }
  return 0;
}

//reads a byte in a specified pipe, returns 0 if successful or 1 if blocked
int read_from_pipe( int i, char* x){
  if(pipes[ i ].readBlock == true) return 1;
  if(next(pipes[ i ].start, pipes[ i ].max) > pipes[ i ].end) {  //checks if pipe is blocked
    pipes[ i ].readBlock = true;                                         //or needs to be blocked
    return 1;
  }
  else{
    *x = ( char )( *pipes[ i ].data[next(pipes[ i ].start, pipes[ i ].max)] );
    pipes[ i ].start = next(pipes[ i ].start, pipes[ i ].max);   //write data and adjust
    if( pipes[ i ].writeBlock ) pipes[ i ].writeBlock = false;       //pointers, unblock write
    return 0;
  }
}

//returns the index of the next available pipe, -1 if full, -2 if name not unique 
int next_pipe( int x ){
  int available;
  for ( int i = 99; i > -1; i-- ){                 //goes through all pipes
    if( pipe_occupied[ i ] == 0 ) available = i;   //checks the pipe is not already in use
    else if( pipes[ i ].name == x ){               //checks if the name is unique
      available = -2;                              //if not returns error
      break;
    }
  }
  if(( available == 0) && (pipe_occupied[ 0 ] == 1)) available = -1; //checks the last pipe is valid
  return available;
}

int pipe_name_to_index( int x ){
  int i;
  for( i = 0 ; i < 100; i++ ){
    if(pipes[ i ].name == x) break;
  }
  if(( i == 100 ) && ( pipes[100].name != x )) i = -1;
  return i;
}

//removes the pipe
void unlink_pipe( int index ){ //need to do something?
  pipes[ index ].name = -1;
  pipe_occupied[ index ] = 0;
}

void hilevel_handler_svc( ctx_t* ctx, uint32_t id ) {
  /* Based on the identified encoded as an immediate operand in the
   * instruction,
   *
   * - read  the arguments from preserved usr mode registers,
   * - perform whatever is appropriate for this system call,
   * - write any return value back to preserved usr mode registers.
   */
  switch( id ) {
    case 0x00 : { // 0x00 => yield()
      scheduler( ctx );
      break;
    }

    case 0x01 : { // 0x01 => write( fd, x, n )
      int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );
      
      int i;
      if(fd == 1){ //STDOUT_FILENO           TODO - fix these (stdlib.c)
        for( i = 0; i < n; i++ ) {
          PL011_putc( UART0, *x++, true );
        }
      }
      else if(( fd > 2 ) && ( fd < 100 )){  //fix pipe numbers
        for( i = 0; i < n; i++){
          if( !write_to_pipe( fd, *x++ )) break;
        }
      }
      else{
        //error?
      }
      
      ctx->gpr[ 0 ] = ( n - i ) - 1; //returns the number of characters unwritten?
      break;
    }
      
    case 0x02 : { // 0x02 => read( fd, x, n )
      int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );
      
      int i;
      if(( fd > 2 ) && ( fd < 100 )){
        for( i = 0; i < n; i++ ) {
          if( !read_from_pipe( fd, x )) break;
        }
      }
      
      ctx->gpr[ 0 ] = ( n - i ) - 1; //return 
      break;
    }
          
    case 0x03 : { // 0x03 => fork()
      int pcb_index = lowestAvailablePcb();
      memset( &pcb[ pcb_index ], 0, sizeof( pcb_t ) ); //initialise PCB 
      memcpy( &pcb[ pcb_index ].ctx, ctx, sizeof( ctx_t )); //copy the current program into a new pcb
      pcb[ pcb_index ].pri_age = 0;
      pcb[ pcb_index ].priority = 1;
      pcb[ pcb_index ].pid = next_pid;
      pcb[ pcb_index ].ctx.gpr[ 0 ] = ( uint32_t ) 0;
      pcb[ pcb_index ].ctx.sp = ( uint32_t ) &tos_PS - ((pcb_index) * 1000); //fix stack pointer
      pcb[ pcb_index ].status = STATUS_READY;
      occupied[ pcb_index ] = 1;
      ctx->gpr[ 0 ] = pcb_index;
      next_pid++;
      break;
    }

    case 0x04 : { // 0x04 => exit(int x)
      int status = ( int ) ctx->gpr[ 0 ];
      if( status == 0 ){
        //exit success TODO SOMETHING HERE?
      }
      else if( status == 1 ){
        //exit failure
      }
      else{
        //error
      }
      terminateProcess( pcb[ executing ].pid );
      break;
    }

    case 0x05 : { // 0x05 => exec(void* x)
      ctx->pc = ctx->gpr[ 0 ];
      break;
    }
          
    case 0x06 : { // 0x06 => kill(pid_t pid, int x)}
      pid_t pid = ( pid_t )( ctx->gpr[ 0 ] );
      int   sig = ( int   )( ctx->gpr[ 1 ] );
      //TODO something with the signal?
      terminateProcess( pid );
      break;
    }
      
    case 0x07 : { // 0x07 => nice(pid_t pid, int x)}
      pid_t pid = ( pid_t )( ctx->gpr[ 0 ] );
      int   pri = ( int   )( ctx->gpr[ 1 ] );
      pcb[ pidToIndex( pid ) ].priority = pri;
      break;
    }
    
    //takes a positive int and creates a pipe with it,
    //errors: 1 for negaitve name, 2 for non unique name, 3 if pipes are full
    case 0x08 : { //0x08 => mkfifo( int x ) todo - change from int?
      int x = ( int )( ctx->gpr[ 0 ] );  //name
      
      if( x < 1 ) ctx->gpr[ 0 ] = 1;     //checks it is valid
      else {
        int pipe_index = next_pipe( x );   //gets the next available pipe
        if( pipe_index > 0 ) {
          memset( &pipes[ pipe_index ], 0, sizeof( pipe_t ) ); //allocate pipe
          pipes[ pipe_index ].name = x;
          pipes[ pipe_index ].writepid = -1;
          pipes[ pipe_index ].readpid = -1;
          pipes[ pipe_index ].start = 0;
          pipes[ pipe_index ].end = 1;
          pipes[ pipe_index ].max = 10;
          pipes[ pipe_index ].readBlock = false;
          pipes[ pipe_index ].writeBlock = false;
          pipe_occupied[ pipe_index ] = 1;
          ctx->gpr[ 0 ] = 0;
        }
        else if( pipe_index == -1 ) ctx->gpr[ 0 ] = 3;
        else ctx->gpr[ 0 ] = 2;
      }
      break;
      //initialise access permissions??
    }
    
    //gives name returns descriptor if successful, -1 if the pipe doesn't exist, 
    //-2 if it is already occupied, -3 for non valid mode
    case 0x09 : { //0x09 => open ( int x, int mode )
      int x = ( int )( ctx->gpr[ 0 ] ); //name
      int mode = ( int )( ctx->gpr[ 1 ]); //mode
      //0 = read, 1 = write
      int index = pipe_name_to_index( x ); //TODO - set to small function?
      ctx->gpr[ 0 ] = index;
      if(index == -1) ctx->gpr[ 0 ] = -1; //check pipe exists
      else {
        if(mode == 0){
          if(pipes[ index ].readpid != -1) ctx->gpr[ 0 ] = -2; //checks pipe is avaliable
          else pipes[ index ].readpid = executing;
        }
        else if(mode == 1){
          if(pipes[ index ].writepid != -1) ctx->gpr[ 0 ] = -2; //checks pipe is avaliable
          else pipes[ index ].writepid = executing;
        }
        else ctx->gpr[ 0 ] = -3;
      }
      break;
    }
      
    //deallocates end of pipe, returns 0 if successful, 1 if pipe doesn't exist or 
    //2 if the process does not connect to it 
    case 0x10 : { //0x10 => close( int fd )
      int fd = ( int )( ctx-> gpr[ 0 ] );
      ctx->gpr[ 0 ] = 0;
      
      if( ( fd > 100 )|| ( fd < 0 ) ) ctx->gpr[ 0 ] = 1; //check valid fd
      else {
        if(pipes[ fd ].readpid == executing) pipes[ fd ].readpid = -1;        //checks process is connected
        else if(pipes[ fd ].writepid == executing) pipes[ fd ].writepid = -1; //if so resets pipes
        else ctx->gpr[ 0 ] = 2;                                                  //otherwise error
      }
      break;
    }
      
    //deallocates pipe, 0 for success, 1 for pipe doesn't exist, 
    //2 for pipe not connected to process, 3 for pipe not deallicated on other side
    case 0x11 : { //0x11 => unlink( int fd )
      int fd = ( int )( ctx->gpr[ 0 ] );
      
      ctx->gpr[ 0 ] = 0;
      if( ( fd > 100 )|| ( fd < 0 ) ) ctx->gpr[ 0 ] = 1; //check valid fd
      else {
        if(pipes[ fd ].readpid == -1){                              //checks one side of the pipe closed
          if(pipes[ fd ].writepid == executing) unlink_pipe( fd );   //and the process is connected to the
          else ctx->gpr[ 0 ] = 3;                                   //other, if so it closes it
        }
        else if(pipes[ fd ].writepid == -1){
          if(pipes[ fd ].readpid == executing) unlink_pipe( fd );
          else ctx->gpr[ 0 ] = 3;
        }
        else ctx->gpr[ 0 ] = 2;
      }
      break;
    }
          
    default   : { // 0x?? => unknown/unsupported
      break;
    }
  }

  return;
}

void hilevel_handler_irq(ctx_t* ctx) {
  // Step 2: read  the interrupt identifier so we know the source.
  uint32_t id = GICC0->IAR;

  // Step 4: handle the interrupt, then clear (or reset) the source.

  if( id == GIC_SOURCE_TIMER0 ) {
    PL011_putc( UART0, '\n', true );
    scheduler(ctx);
    TIMER0->Timer1IntClr = 0x01;
  }

  // Step 5: write the interrupt identifier to signal we're done.

  GICC0->EOIR = id;

  return;
}
