<template>
  <div>
    <!-- <h4>{{ title }}</h4> -->

    <!-- <ul>
      <li>
        Not Cakes
      </li>
    </ul> -->

    <!-- <yummy_cakes /> -->
  
    <webapp_header /> 
    <webapp_content /> 
    <webapp_footer /> 

  </div>
</template>

<script>
  // import yummy_cakes from './components/yummy_cakes.vue';
  import header from './views/WebHeader.vue';
  import content from './views/WebContent.vue';
  import footer from './views/WebFooter.vue';
  
  export default 
  {
    name: 'App',

    components: {
      // 'yummy_cakes': yummy_cakes
      'webapp_header': header,
      'webapp_content': content,
      'webapp_footer': footer,

    },

    data () {
      return {
        title: 'Your first title ;)'
      }
    }
  }
</script>
